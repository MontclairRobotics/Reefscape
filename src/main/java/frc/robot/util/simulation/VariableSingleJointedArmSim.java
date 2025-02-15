// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.simulation;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Represents a simulated single jointed arm mechanism. */
public class VariableSingleJointedArmSim extends SingleJointedArmSim {

  private Supplier<Double> m_armLenMetersSupplier;
  private boolean m_simulateGravity;
  private double m_minAngle;
  private double m_maxAngle;

  /**
   * Creates a simulated arm mechanism.
   *
   * @param plant The linear system that represents the arm. This system can be created with {@link
   *     edu.wpi.first.math.system.plant.LinearSystemId#createSingleJointedArmSystem(DCMotor,
   *     double, double)}.
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param armLengthMeters The length of the arm.
   * @param minAngleRads The minimum angle that the arm is capable of.
   * @param maxAngleRads The maximum angle that the arm is capable of.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingAngleRads The initial position of the Arm simulation in radians.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  @SuppressWarnings("this-escape")
  public VariableSingleJointedArmSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      Supplier<Double> armLengthMetersSupplier,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity,
      double startingAngleRads,
      double... measurementStdDevs) {
    super(plant, gearbox, gearing, armLengthMetersSupplier.get(), minAngleRads, maxAngleRads, simulateGravity, startingAngleRads, measurementStdDevs);
    m_armLenMetersSupplier = armLengthMetersSupplier;

    // Annoyingly this variables are private in SingleJointedArm
    // so we need to replicate here
    m_simulateGravity = simulateGravity;
    m_minAngle = minAngleRads;
    m_maxAngle = maxAngleRads;
  }

  /**
   * Creates a simulated arm mechanism.
   *
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the arm; can be calculated from CAD software.
   * @param armLengthMeters The length of the arm.
   * @param minAngleRads The minimum angle that the arm is capable of.
   * @param maxAngleRads The maximum angle that the arm is capable of.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingAngleRads The initial position of the Arm simulation in radians.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public VariableSingleJointedArmSim(
      DCMotor gearbox,
      double gearing,
      double jKgMetersSquared,
      Supplier<Double> armLengthMetersSupplier,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity,
      double startingAngleRads,
      double... measurementStdDevs) {
    this(
        LinearSystemId.createSingleJointedArmSystem(gearbox, jKgMetersSquared, gearing),
        gearbox,
        gearing,
        armLengthMetersSupplier,
        minAngleRads,
        maxAngleRads,
        simulateGravity,
        startingAngleRads,
        measurementStdDevs);
  }


  /**
   * Updates the state of the arm.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dtSeconds The time difference between controller updates.
   */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // Annoyingly the arm length variable in SingleJointedArm is private
    // so we can't just set that and call the super method like this:
    // 
    //   m_armLenMeters  = m_armLenMetersSupplier.get();
    //   super.updateX(currentXhat, u, dtSeconds);
    // 
    // Instead we replicate the entire method here


    // The torque on the arm is given by τ = F⋅r, where F is the force applied by
    // gravity and r the distance from pivot to center of mass. Recall from
    // dynamics that the sum of torques for a rigid body is τ = J⋅α, were τ is
    // torque on the arm, J is the mass-moment of inertia about the pivot axis,
    // and α is the angular acceleration in rad/s². Rearranging yields: α = F⋅r/J
    //
    // We substitute in F = m⋅g⋅cos(θ), where θ is the angle from horizontal:
    //
    //   α = (m⋅g⋅cos(θ))⋅r/J
    //
    // Multiply RHS by cos(θ) to account for the arm angle. Further, we know the
    // arm mass-moment of inertia J of our arm is given by J=1/3 mL², modeled as a
    // rod rotating about it's end, where L is the overall rod length. The mass
    // distribution is assumed to be uniform. Substitute r=L/2 to find:
    //
    //   α = (m⋅g⋅cos(θ))⋅r/(1/3 mL²)
    //   α = (m⋅g⋅cos(θ))⋅(L/2)/(1/3 mL²)
    //   α = 3/2⋅g⋅cos(θ)/L
    //
    // This acceleration is next added to the linear system dynamics ẋ=Ax+Bu
    //
    //   f(x, u) = Ax + Bu + [0  α]ᵀ
    //   f(x, u) = Ax + Bu + [0  3/2⋅g⋅cos(θ)/L]ᵀ
    Matrix<N2, N1> updatedXhat =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              if (m_simulateGravity) {
                double alphaGrav = 3.0 / 2.0 * -9.8 * Math.cos(x.get(0, 0)) / m_armLenMetersSupplier.get();
                xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
              }
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collision after updating xhat
    if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_minAngle, 0);
    }
    if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_maxAngle, 0);
    }
    return updatedXhat;
  }




  /**
   * This returns the effective lenth to the COM of the arm based on the pivot angle of the wrist 
   * 
   * @param foreArmLengthMeters - Full length of the fore arm part of double pivot
   * @param foreArmLengthToCOMMeters - Length to COM of fore arm (if evenly weighted, this is just foreArmLengthMeters/2)
   * @param foreArmMassKg - Mass of the fore arm
   * @param handLengthToCOMMeters - Length to COM of hand part of double pivot
   * @param handMassKg - Mass of the hand
   * @param wristAngleSupplier - Angle of write (between fore arm and hand)
   * @return
   */
  public static Supplier<Double> getDoublePivotLengthSupplier(double foreArmLengthMeters, double foreArmLengthToCOMMeters, double foreArmMassKg, double handLengthToCOMMeters, double handMassKg, Supplier<Double> wristAngleSupplier) {
    return () -> {
      // Length of hand relative to horizontal
      double effectiveHandLength = Math.cos(wristAngleSupplier.get()) * handLengthToCOMMeters;
      // Combine COM of fore arm with effective COM of hand
      double effectivelength = ((foreArmLengthToCOMMeters * foreArmMassKg + (foreArmLengthMeters + effectiveHandLength) * handMassKg)) / (foreArmMassKg + handMassKg);
      return effectivelength;
    };
  }
}

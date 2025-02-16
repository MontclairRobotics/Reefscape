/**
Copyright 2023 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with SpartanLib2. 
If not, see <https://www.gnu.org/licenses/>.
*/
package frc.robot.util.simulation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
// import org.chsrobotics.lib.math.UtilityMath;

/**
 * Combined feedforward and simulation model of a double-jointed arm.
 *
 * <p>Derived from https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060 and
 * https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/arm/ArmDynamics.java
 */
public class DoubleJointedArmModel {
    private final double localMass;
    private final double localCGRadius;
    private final double localMoment;
    private final double localLength;

    private final DCMotor localDrive;

    private final double distalMass;
    private final double distalCGRadius;
    private final double distalMoment;

    private final DCMotor distalDrive;

    private final double kG;

    /**
     * Constructs a new DoubleJointedArmModel.
     *
     * @param localMass Mass of the local arm segment, in kg.
     * @param localCGRadius Length from the root of the local arm to its center of gravity, in
     *     meters.
     * @param localMoment Moment of inertia of the local arm about its center of gravity, in kg m^2.
     * @param localLength Length of the local arm segment, in meters.
     * @param localDrive A DCMotor to power the local arm segment. Should include any reductions.
     * @param distalMass Mass of the distal arm segment, in kg.
     * @param distalCGRadius Length from the root of the distal arm to its center of gravity, in
     *     meters.
     * @param distalMoment Moment of inertia of the distal arm about its center of gravity, in kg
     *     m^2.
     * @param distalDrive A DCMotor to power the distal arm segment. Should include any reductions.
     * @param kG A scalar scaling factor to apply to gravitational torques.
     */
    public DoubleJointedArmModel(
            double localMass,
            double localCGRadius,
            double localMoment,
            double localLength,
            DCMotor localDrive,
            double distalMass,
            double distalCGRadius,
            double distalMoment,
            DCMotor distalDrive,
            double kG) {
        this.localMass = localMass;
        this.localCGRadius = localCGRadius;
        this.localMoment = localMoment;
        this.localLength = localLength;

        this.localDrive = localDrive;

        this.distalMass = distalMass;
        this.distalCGRadius = distalCGRadius;
        this.distalMoment = distalMoment;

        this.distalDrive = distalDrive;

        this.kG = kG;
    }

    /**
     * Computes feedforward motor voltages for a given arm state.
     *
     * @param position A Vector representing arm joint angles in radians, as [local, distal].
     * @param velocity A Vector representing arm angular velocities in rad/s, as [local, distal].
     * @param acceleration A Vector representing arm angular accelerations in rad/s^2, as [local,
     *     distal].
     * @return A Vector of feedforward motor voltages, as [local, distal].
     */
    public Vector<N2> feedforward(
            Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
        Matrix<N2, N1> torque =
                inertiaMatrix(position)
                        .times(acceleration)
                        .plus(cMatrix(position, velocity).times(velocity))
                        .plus(gravityMatrix(position));

        return VecBuilder.fill(
                localDrive.getVoltage(torque.get(0, 0), velocity.get(0, 0)),
                distalDrive.getVoltage(torque.get(1, 0), velocity.get(1, 0)));
    }

    /**
     * Computes feedforward motor voltages for a given arm state and no acceleration.
     *
     * @param position A Vector representing arm joint angles in radians, as [local, distal].
     * @param velocity A Vector representing arm angular velocities in rad/s, as [local, distal].
     * @return A Vector of feedforward motor voltages, as [local, distal].
     */
    public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity) {
        return feedforward(position, velocity, VecBuilder.fill(0, 0));
    }

    /**
     * Computes feedforward motor voltages for a given arm state with no acceleration or velocity.
     *
     * @param position A Vector representing arm joint angles in radians, as [local, distal].
     * @return A Vector of feedforward motor voltages, as [local, distal].
     */
    public Vector<N2> feedforward(Vector<N2> position) {
        return feedforward(position, VecBuilder.fill(0, 0));
    }

    /**
     * Simulates the arm system by numerical integration.
     *
     * @param state A Matrix of arm state, as
     *     <p>[localAngle, localVelocity]
     *     <p>[distalAngle, distalVelocity]
     * @param voltages A Vector of input voltages, as [local, distal].
     * @param dtSeconds The timestep of the simulated system, in seconds.
     * @return A Matrix of arm state after the simulated timestep, as
     *     <p>[localAngle, localVelocity]
     *     <p>[distalAngle, distalVelocity]
     */
    public Matrix<N2, N2> simulate(Matrix<N2, N2> state, Vector<N2> voltages, double dtSeconds) {
        Vector<N4> stateAsVector =
                VecBuilder.fill(state.get(0, 0), state.get(0, 1), state.get(1, 0), state.get(1, 1));

        Vector<N2> clampedVoltage =
                VecBuilder.fill(
                        clamp(localDrive.nominalVoltageVolts, voltages.get(0, 0)),
                        clamp(distalDrive.nominalVoltageVolts, voltages.get(1, 0)));
        Matrix<N4, N1> prelim =
                NumericalIntegration.rkdp(
                        this::instDynamics, stateAsVector, clampedVoltage, dtSeconds);

        Matrix<N2, N2> mat = new Matrix<>(N2.instance, N2.instance);

        mat.set(0, 0, prelim.get(0, 0));
        mat.set(0, 1, prelim.get(1, 0));
        mat.set(1, 0, prelim.get(2, 0));
        mat.set(1, 1, prelim.get(3, 0));

        return mat;
    }

    /**
     * Constrains a value under a maximum absolute value.
     *
     * @param maxValueAbs The maximum allowed absolute value of the output.
     * @param value The value to constrain.
     * @return The constrained value.
     */
    public static double clamp(double maxValueAbs, double value) {
        return clamp(Math.abs(maxValueAbs), -Math.abs(maxValueAbs), value);
    }

    public static double clamp(double ceiling, double floor, double value) {
        if (value > ceiling) return ceiling;
        else if (value < floor) return floor;
        else return value;
    }

    private Matrix<N4, N1> instDynamics(Matrix<N4, N1> x, Matrix<N2, N1> u) {
        Matrix<N4, N1> mat = new Matrix<>(N4.instance, N1.instance);

        Vector<N2> positions = VecBuilder.fill(x.get(0, 0), x.get(1, 0));

        Vector<N2> velocities = VecBuilder.fill(x.get(2, 0), x.get(3, 0));

        Vector<N2> torques =
                VecBuilder.fill(
                        localDrive.getTorque(
                                localDrive.getCurrent(velocities.get(0, 0), u.get(0, 0))),
                        distalDrive.getTorque(
                                distalDrive.getCurrent(velocities.get(1, 0), u.get(1, 0))));

        Matrix<N2, N1> accel =
                inertiaMatrix(positions)
                        .inv()
                        .times(
                                torques.minus(cMatrix(positions, velocities).times(velocities))
                                        .minus(gravityMatrix(positions)));

        mat.set(0, 0, velocities.get(0, 0));
        mat.set(1, 0, velocities.get(1, 0));

        mat.set(2, 0, accel.get(0, 0));
        mat.set(3, 0, accel.get(1, 0));

        return mat;
    }

    private Matrix<N2, N2> inertiaMatrix(Vector<N2> position) {
        Matrix<N2, N2> mat = new Matrix<>(N2.instance, N2.instance);

        mat.set(
                0,
                0,
                (localMass * localCGRadius * localCGRadius)
                        + (distalMass * (Math.pow(localLength, 2) + Math.pow(distalCGRadius, 2)))
                        + localMoment
                        + distalMoment
                        + (2
                                * distalMass
                                * localLength
                                * distalCGRadius
                                * Math.cos(position.get(1, 0))));

        mat.set(
                1,
                0,
                (distalMass * distalCGRadius * distalCGRadius)
                        + distalMoment
                        + (distalMass
                                * localLength
                                * distalCGRadius
                                * Math.cos(position.get(1, 0))));

        mat.set(
                0,
                1,
                (distalMass * distalCGRadius * distalCGRadius)
                        + distalMoment
                        + (distalMass * distalCGRadius * Math.cos(position.get(1, 0))));

        mat.set(1, 1, (distalMass * distalCGRadius * distalCGRadius) + distalMoment);

        return mat;
    }

    private Matrix<N2, N2> cMatrix(Vector<N2> positions, Vector<N2> velocities) {
        Matrix<N2, N2> mat = new Matrix<>(N2.instance, N2.instance);

        mat.set(0, 0, -distalMass * localLength * distalCGRadius * velocities.get(1, 0));

        mat.set(
                0,
                1,
                -distalMass
                        * localLength
                        * distalCGRadius
                        * Math.sin(positions.get(1, 0))
                        * (velocities.get(0, 0) + velocities.get(1, 0)));

        mat.set(
                1,
                0,
                distalMass
                        * localLength
                        * distalCGRadius
                        * Math.sin(positions.get(1, 0))
                        * velocities.get(1, 0));

        mat.set(1, 1, 0);

        return mat;
    }

    private Matrix<N2, N1> gravityMatrix(Vector<N2> positions) {
        Matrix<N2, N1> mat = new Matrix<>(N2.instance, N1.instance);

        double g = 9.81 * 10;
        // compensates for odd timestep stuff, obviously arbitrary

        mat.set(
                0,
                0,
                g
                        * (Math.cos(positions.get(0, 0))
                                        * ((localMass * localCGRadius)
                                                + (distalMass + distalCGRadius))
                                + (distalMass
                                        * distalCGRadius
                                        * Math.cos(positions.get(0, 0) + positions.get(1, 0)))));

        mat.set(
                1,
                0,
                g
                        * distalMass
                        * distalCGRadius
                        * Math.cos(positions.get(0, 0) + positions.get(1, 0)));

        return mat.times(kG);
    }
}
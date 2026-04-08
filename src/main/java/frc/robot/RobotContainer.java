// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RollersConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.RollersIOSim;
import frc.robot.subsystems.intake.rollers.RollersIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.spindexer.indexer.Indexer;
import frc.robot.subsystems.shooter.spindexer.indexer.IndexerIOSim;
import frc.robot.subsystems.shooter.spindexer.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.spindexer.serializer.Serializer;
import frc.robot.subsystems.shooter.spindexer.serializer.SerializerIOSim;
import frc.robot.subsystems.shooter.spindexer.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.PoseUtils;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import frc.robot.util.sim.FuelSim;
import frc.robot.util.tunables.LoggedTunableNumber;
import frc.robot.util.tunables.Tunable;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;


import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;

public class RobotContainer {

	// Controllers
	public static CommandPS5Controller driverController = new CommandPS5Controller(0);
	public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
	public static CommandPS5Controller testController = new CommandPS5Controller(2);

	// Subsystems
	public static Vision vision;
	public static CommandSwerveDrivetrain drivetrain;

	// shooter
	public static Shooter shooter;
	public static Turret turret;
	public static Flywheel flywheel;
	public static Hood hood;

	public static Spindexer spindexer;
	public static Serializer serializer;
	public static Indexer indexer;

	// intake
	public static Pivot pivot;
	public static Rollers rollers;
	public static Intake intake;

	public static Superstructure superstructure;

	public static Auto auto;

	public static SimShootingParameters simShootingParameters = new SimShootingParameters(Degrees.zero(), Degrees.zero(), MetersPerSecond.zero());

	private SwerveDriveSimulation driveSimulation;
	private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED.in(MetersPerSecond));
	public static FuelSim fuelSim = new FuelSim("fuel");

	private boolean useConstantVelocityMap = false;
	private boolean shootWhileMoving = true;

	// debug, set to true to increase logging, set to false to increase performance and reduce loop overruns
	public static boolean VISION_DEBUG = false;
	public static boolean TURRET_DEBUG = false;
	public static boolean FLYWHEEL_DEBUG = false;
	public static boolean HOOD_DEBUG  = false;
	public static boolean INDEXER_DEBUG = false;
	public static boolean SERIALIZER_DEBUG = false;
	public static boolean ROLLERS_DEBUG = false;
	public static boolean PIVOT_DEBUG = false;
	public static boolean DRIVETRAIN_DEBUG = false;
	public static boolean SUPERSTRUCTURE_DEBUG = false;

	public double turretFudge = 0;

	public static boolean shouldShootAuto = false;

	public static Trigger shootButtonTrigger = operatorController.circle();

	public static Trigger robotRelativeTrigger = driverController.L1();
	public static Trigger xModeTrigger = driverController.R1();
	public static Trigger turboTrigger = driverController.L2();
	public static Trigger precisionTrigger = driverController.R2();

	public LoggedTunableNumber indexerCurrent = new LoggedTunableNumber("Spindexer/Index Current", 0);
	public LoggedTunableNumber serializerCurrent = new LoggedTunableNumber("Spindexer/Serializer Current", 0);

	public LoggedTunableNumber indexerVelocity = new LoggedTunableNumber("Spindexer/Index Velocity", 0);
	public LoggedTunableNumber serializerVelocity = new LoggedTunableNumber("Spindexer/Serializer Velocity", 0);

	public LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Intake/Intake Voltage", RollersConstants.SPIN_VOLTAGE);

	public RobotContainer() {

		Tunable turretFudgeTunable = new Tunable("Turret Fudge", turretFudge, (value) -> TurretConstants.ANGLE_OFFSET = Rotations.of(0.375).plus(Degrees.of(value)));

		System.out.println("Constants.CURRENT_MODE: " + Constants.CURRENT_MODE);

		switch (Constants.CURRENT_MODE) {
			case REAL:
				drivetrain = TunerConstants.createDrivetrain();

				hood = new Hood(new HoodIOTalonFX());
				flywheel = new Flywheel(new FlywheelIOTalonFX());
				turret = new Turret(new TurretIOTalonFX());

				serializer = new Serializer(new SerializerIOTalonFX());
				indexer = new Indexer(new IndexerIOTalonFX());
				spindexer = new Spindexer(serializer, indexer);

				shooter = new Shooter(
					hood, flywheel, turret, spindexer,
					useConstantVelocityMap, shootWhileMoving
				);

				pivot = new Pivot(new PivotIOTalonFX());
				rollers = new Rollers(new RollersIOTalonFX());
				intake = new Intake(pivot, rollers);

				auto = new Auto();

				vision = new Vision(
					drivetrain::addVisionMeasurement,
					new VisionIOLimelight(camera0Name, () -> drivetrain.odometryHeading),
					// new VisionIOLimelight(camera1Name, () -> drivetrain.odometryHeading),
					new VisionIOLimelight(camera2Name, () -> drivetrain.odometryHeading)
				);

				superstructure = new Superstructure(drivetrain, intake, shooter, vision);

				break;

			case SIM:
				drivetrain = TunerConstants.createDrivetrain();
				driveSimulation = drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive;

				hood = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());
				turret = new Turret(new TurretIOSim());

				serializer = new Serializer(new SerializerIOSim());
				indexer = new Indexer(new IndexerIOSim());
				spindexer = new Spindexer(serializer, indexer);

				shooter = new Shooter(
					hood, flywheel, turret, spindexer,
					useConstantVelocityMap, shootWhileMoving
				);

				pivot = new Pivot(new PivotIOSim());
				rollers = new Rollers(new RollersIOSim());
				intake = new Intake(pivot, rollers);

				fuelSim.enableAirResistance();
				fuelSim.start();

				fuelSim.registerRobot(
					Constants.BUMPER_WIDTH,
					Constants.BUMPER_WIDTH,
					Inches.of(6),
					() -> drivetrain.getRobotPose(),
					() -> drivetrain.getFieldRelativeSpeeds()
				);

				fuelSim.registerIntake(
					Inches.of(15),
					Inches.of(22),
					Inches.of(-15),
					Inches.of(15),
					shooter::shouldIntake,
					shooter::addBall
				);

				fuelSim.spawnStartingFuel();

				auto = new Auto();
				superstructure = new Superstructure(drivetrain, intake, shooter, vision);

				break;

				default:
					vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
		}

		// configureBindings();
		configureCompetitionBindings();
		// configureTestingBindingsForMax();

    	drivetrain.registerTelemetry(logger::telemeterize);
	}

	private void configureTestingBindingsForMax() {

		// runs the spindexer + indexer at max speed

		operatorController.circle()
			.whileTrue(
				spindexer.spinUpCommand()
					.alongWith(
						flywheel.setVelocityCommand(
							RotationsPerSecond.of(20), () -> Timer.getFPGATimestamp()
						)
					)
			)
			.onFalse(
				spindexer.spinDownCommand()
					.alongWith(
						flywheel.stopCommand()
					)
			);

	}

	private void configureCompetitionBindings() {

		driverController.povRight().whileTrue(new WheelRadiusCharacterization(Direction.CLOCKWISE, drivetrain));
		driverController.povLeft().whileTrue(new WheelRadiusCharacterization(Direction.COUNTER_CLOCKWISE, drivetrain));

		// driver
		drivetrain.setDefaultCommand(new JoystickDriveCommand(false));
		driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());
		driverController.PS().onTrue(drivetrain.resetPoseCommand(new Pose2d(3.6, 4.035, new Rotation2d())));
		precisionTrigger
			.onTrue(drivetrain.setMaxSpeedsCommand(MetersPerSecond.of(1.5), RotationsPerSecond.of(0.3)))
			.onFalse(drivetrain.setMaxSpeedsCommand(TunerConstants.kSpeedAt12Volts, RotationsPerSecond.of(1.624)));

		driverController.triangle()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
		driverController.square()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(90)), false));
		driverController.cross()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
		driverController.circle()
			.onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(-90)), false));

		// operator

		operatorController.touchpad().whileTrue(spindexer.setVoltageCommand(-12)).onFalse(spindexer.spinDownCommand());
		operatorController.PS().whileTrue(rollers.setVoltageCommand(-12)).onFalse(serializer.stopCommand());

		operatorController.circle().onFalse(shooter.stowCommand());

		operatorController.povLeft().onTrue(turret.increaseFudgeFactorCommand());
		operatorController.povRight().onTrue(turret.decreaseFudgeFactorCommand());

		operatorController.L1().whileTrue(pivot.deployCommand().alongWith(rollers.setVoltageCommand(() -> intakeVoltage.get()))).onFalse(pivot.stopCommand().alongWith(rollers.setVoltageCommand(() -> 0)));
		operatorController.L2().whileTrue(spindexer.spinUpCommand()).onFalse(spindexer.spinDownCommand());

		operatorController.povUp().onTrue(Commands.runOnce(() -> flywheel.increaseFudge()));
		operatorController.povDown().onTrue(Commands.runOnce(() -> flywheel.decreaseFudge()));

		operatorController.R1().whileTrue(pivot.stowCommand()).onFalse(pivot.stopCommand());
		operatorController.R2()
			.whileTrue(intake.jiggleCommand())
			.onFalse(pivot.deployCommand());

	}

	private void configureBindings() {

		driverController.povRight().whileTrue(new WheelRadiusCharacterization(Direction.CLOCKWISE, drivetrain));
		driverController.povLeft().whileTrue(new WheelRadiusCharacterization(Direction.COUNTER_CLOCKWISE, drivetrain));

		operatorController.circle().onFalse(shooter.stowCommand());
		drivetrain.setDefaultCommand(new JoystickDriveCommand(false));
		driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());

		driverController.triangle().whileTrue(
			indexer.setVelocityCommand(() -> RotationsPerSecond.of(indexerVelocity.getAsDouble()))
			.alongWith(serializer.setVelocityCommand(() -> RotationsPerSecond.of(serializerVelocity.getAsDouble())))
		).onFalse(
			spindexer.spinDownCommand()
		);

		operatorController.R1().whileTrue(pivot.stowCommand()).onFalse(pivot.stopCommand());
		operatorController.L1().whileTrue(pivot.deployCommand().alongWith(rollers.spinUpCommand())).onFalse(pivot.stopCommand().alongWith(rollers.setVoltageCommand(() -> 0)));

		operatorController.cross().onTrue(turret.lockForever());

		driverController.R1().whileTrue(spindexer.spinUpCommand()).onFalse(spindexer.spinDownCommand());

		driverController.square()
			.whileTrue(hood.setAngleCommand(() -> Degrees.of(hood.tunableHoodAngle.get()), () -> Timer.getFPGATimestamp()))
			.onFalse(hood.stopCommand());

		driverController.circle()
			.whileTrue(flywheel.setVelocityCommand(() -> RotationsPerSecond.of(flywheel.tuningFlywheelSpeed.get()), () -> Timer.getFPGATimestamp()))
			.onFalse(flywheel.stopCommand());

		if(Constants.CURRENT_MODE == Mode.SIM) driverController.PS().whileTrue(Commands.runOnce(() -> fuelSim.clearFuel()));

	}

	public Command getAutonomousCommand() {
		// System.out.println(auto.getAutoCommand());
		return auto.getAutoCommand();
		// return Commands.none();
	}

	/**
	 * Resets the simulation.
	 *
	 * <p>Borrowed from
	 * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java#L394.
	 */
	public void resetSimulation() {
		if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;
		drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	/** Updates Simulated Arena; to be called from Robot.simulationPeriodic() */
	public void displaySimFieldToAdvantageScope() {
		if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

		SimulatedArena.getInstance().simulationPeriodic();
		// The pose by maplesim, including collisions with the field.
		// See https://www.chiefdelphi.com/t/simulated-robot-goes-through-walls-with-maplesim/508663.
		Logger.recordOutput(
			"FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
		Logger.recordOutput("ferry1",new Translation2d(2,7));
		// Logger.recordOutput("ferry2",new Translation2d(2,1));
		Logger.recordOutput("ferrydistance", turret.getDistanceToPoint(new Translation2d(2,7)));
	}
}
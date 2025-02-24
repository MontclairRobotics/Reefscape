// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignToReefTagCommand;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.leds.LEDs;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.util.RobotState;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;
import frc.robot.util.GamePiece;
import frc.robot.util.PoseUtils;
import frc.robot.util.TagOffset;
import frc.robot.util.TunerConstants;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightSim;
import frc.robot.vision.LimelightSim.LimelightModel;
import frc.robot.vision.LimelightSim.LimelightResolution;


public class RobotContainer {

  //Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static CommandPS5Controller testingController = new CommandPS5Controller(2);

  public static final boolean debugMode = true;

  //Subsystems
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();
  public static Limelight bottomLimelight = new Limelight("Limelight-Bottom", Units.inchesToMeters(5), -5, Units.inchesToMeters(13.5), 0);
  public static Limelight topLimelight = new Limelight("Limelight-Top", 0, 0, 0, 0);
  public static LimelightSim limelightSim;
  public static LEDs leds = new LEDs();
  public static Rollers rollers = new Rollers();
  public static Orchestra orchestra = new Orchestra();
  public static Arm arm = new Arm();
  public static Auto auto = new Auto();
  public static Telemetry telemetryLogger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  //Alliance
  public static boolean isBlueAlliance;


  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();

    // Setup limelight sim
    if (Robot.isSimulation())
      limelightSim = new LimelightSim(() -> drivetrain.getRobotPose());

      // Add bottom limelight to sim
      limelightSim.addCamera(bottomLimelight.getRobotToCamera(), "Limelight-Bottom", LimelightModel.LIMELIGHT_4, LimelightResolution.RESOLUTION_720x480, 20, 30);
  }

  private void configureBindings() {

    orchestra.addInstrument(elevator.leftTalonFX);
    orchestra.addInstrument(elevator.rightTalonFX);
    // operatorController.R2().onTrue(playMusic("SevenNationArmy.chrp")).onFalse(stopMusic());

    /*     Default commands */
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());
    rollers.setDefaultCommand(rollers.getDefaultCommand());
    elevator.setDefaultCommand(Commands.run(() -> {
      RobotState pos = RobotState.getDefaultForPiece(rollers.getHeldPiece());
      System.out.println(pos.getHeight());
      elevator.setExtension(pos.getHeight());
      }
    , elevator));
    // elevator.setDefaultCommand(
    //   RobotContainer.elevator.setExtensionCommand(0)
    // );

    arm.setDefaultCommand(Commands.run(() -> {
        RobotState pos = RobotState.getDefaultForPiece(rollers.getHeldPiece());
        arm.setEndpointAngle(pos.getAngle());
      }
    , arm));
    // elevator.setDefaultCommand(elevator.joystickControlCommand());
    // arm.setDefaultCommand(arm.joystickControlCommand());
    // elevator.setDefaultCommand(elevator.joystickControlCommand());
   //leds.setDefaultCommand(leds.playPatternCommand(LEDs.breathingPattern()));
    // operatorController.square().whileTrue(arm.goToAngleCommand(Rotation2d.fromDegrees(30)));

    /* Operator bindings */

    //elevator height commands
    operatorController.L1().whileTrue(elevator.joystickControlCommand()).whileTrue(arm.joystickControlCommand()).onFalse(Commands.runOnce(() -> {
      elevator.stop();
      arm.stopMotor();
    }));
    // operatorController.triangle().whileTrue(Commands.run(() -> elevator.setHeight(1.7), elevator)); //L1 //66.93 inches
    

    //Intake coral
    // operatorController.L2()
    //   .whileTrue(
    //     elevator.setScoringHeightCommand(RobotState.Intake)
    //     .alongWith(arm.goToLocationCommand(RobotState.Intake))
    //     .alongWith(rollers.intakeCoralCommand()))
    //   .onFalse(rollers.stop());

    //Intake algae
    operatorController.R2()
      .whileTrue(rollers.outtakeAlgaeCommand())
      .onFalse(rollers.stopCommand());
    operatorController.R1()
      .whileTrue(rollers.intakeAlgaeCommand())
      .onFalse(rollers.stopCommand());

    
    //Arm coast mode
    testingController.circle().onTrue(arm.setIdleModeCommand(IdleMode.kCoast)).onFalse(arm.setIdleModeCommand(IdleMode.kBrake));
    
    //Coral intake outtake
    operatorController.L1()
      .whileTrue(rollers.intakeCoralCommand())
      .onFalse(rollers.stopCommand());
    operatorController.L2()
      .whileTrue(rollers.outtakeCoralCommand())
      .onFalse(rollers.stopCommand());


    /* SETS DIFFERENT ROBOT STATES */

    operatorController.povDown().whileTrue(arm.setState(RobotState.Intake)).onFalse(arm.stopCommand());
    
    //L3


    // operatorController.triangle().whileTrue(arm.setState(RobotState.L4));
    // operatorController.square().whileTrue(arm.setState(RobotState.Intake));

    operatorController.triangle()
    .whileTrue((arm.setState(RobotState.L3))
    .alongWith(elevator.setState(RobotState.L3)))
    .onFalse((arm.stopCommand())
    .alongWith(elevator.stopCommand()));
    //L4
    operatorController.circle()
    .whileTrue(arm.setState(RobotState.L4).alongWith(elevator.setState(RobotState.L4)))
    .onFalse(arm.stopCommand().alongWith(elevator.stopCommand()));
   // L1
    operatorController.cross()
    .whileTrue(arm.setState(RobotState.L1).alongWith(elevator.setState(RobotState.L1)))
    .onFalse(arm.stopCommand().alongWith(elevator.stopCommand()));
   // L2

    operatorController.square()
    .whileTrue(arm.setState(RobotState.L2).alongWith(elevator.setState(RobotState.L2)))
    .onFalse(arm.stopCommand().alongWith(elevator.stopCommand()));
  
    
    // operatorController.circle().whileTrue(Commands.sequence(
    //   Commands.runOnce(() -> SignalLogger.start()),
    //   elevator.sysIdDynamic(Direction.kForward).until(elevator::isAtTop),
    //   elevator.sysIdDynamic(Direction.kReverse).until(elevator::isAtBottom),
    //   elevator.sysIdQuasistatic(Direction.kForward).until(elevator::isAtTop),
    //   elevator.sysIdQuasistatic(Direction.kReverse).until(elevator::isAtBottom),
    //   Commands.runOnce(() -> SignalLogger.stop())
    // ).onlyWhile(() -> {
    //   return elevator.isSysIDSafe();
    // }));

    //resets elevator encoders
    operatorController.touchpad().onTrue(
      Commands.runOnce(() -> elevator.resetEncoders(0))
      .ignoringDisable(true)
    );

    //Coast mode elevator
    testingController.cross()
      .onTrue(
        Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Coast))
        .ignoringDisable(true)
      )
      .onFalse(
        Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Brake))
        .ignoringDisable(true)
      );

    /* DRIVER BINDINGS */

    //alignment buttons
    driverController.R2().whileTrue(new GoToPoseCommand(TagOffset.CENTER)
     // .andThen(new AlignToAprilTagCommand(ScoreDirection.CENTER))
     );
    driverController.L1().whileTrue(new GoToPoseCommand(TagOffset.LEFT)
    //  .andThen(new AlignToAprilTagCommand(ScoreDirection.LEFT))
    );
    driverController.R1().whileTrue(new GoToPoseCommand(TagOffset.RIGHT)
    //  .andThen(new AlignToAprilTagCommand(ScoreDirection.RIGHT))
    );

    //Robot relative
    driverController.L2()
      .onTrue(drivetrain.toRobotRelativeCommand())
      .onFalse(drivetrain.toFieldRelativeCommand());

    //90 degree buttons
    driverController.triangle()
       .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
    driverController.square()
      .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(90)), false));
    driverController.cross()
      .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
    driverController.circle()
      .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(270)), false)); 
    
    //zeros gyro
    driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());
    drivetrain.registerTelemetry(telemetryLogger::telemeterize);    

  }

  /* MUSIC */
  public void loadMusic(String filepath) {
    //attempt to load music
    var status = orchestra.loadMusic(filepath);
    //send error if it doesn't load
    if(!status.isOK()) {
      Elastic.sendNotification(new Notification(
        NotificationLevel.WARNING, "Music not loading",
        "",
        5000
      ));
    }
  }

  public Command playMusic(String filepath) {
    return Commands.runOnce(() -> {
      loadMusic(filepath);
      orchestra.play();
    });
  }

  public Command stopMusic() {
    return Commands.runOnce(() -> orchestra.stop());
  }

  public Command getAutonomousCommand() {
    System.out.println(auto.getAutoCommand());
    return auto.getAutoCommand();
  }
}
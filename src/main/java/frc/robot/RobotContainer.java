// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.leds.LEDControl;
import frc.robot.leds.LEDs;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.util.ArmPosition;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.GamePiece;
import frc.robot.util.TunerConstants;
import frc.robot.vision.Limelight;


public class RobotContainer {

  //Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static CommandPS5Controller testingController = new CommandPS5Controller(2);

  public static final boolean debugMode = true;

  //Subsystems
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();
  public static Limelight limelight = new Limelight("Camera");
  public static LEDControl ledControl = new LEDControl();
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

  }

  private void configureBindings() {

    /* NAMED COMMANDS */
        NamedCommands.registerCommand(
            "elevator", 
            elevator.setHeightCommand(auto.autoElevatorHeight)
        );

        //TODO: Make this actually move the arm
        NamedCommands.registerCommand(
            "arm",
            Commands.none()
        );

    orchestra.addInstrument(elevator.leftTalonFX);
    orchestra.addInstrument(elevator.rightTalonFX);
    operatorController.R2().onTrue(playMusic("SevenNationArmy.chrp")).onFalse(stopMusic());

    /*     Default commands */
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());
    elevator.setDefaultCommand(Commands.run(() -> {
      ArmPosition pos = ArmPosition.getDefaultForPiece(rollers.getHeldPiece());
        elevator.setScoringHeight(pos);
      }
    ));

    arm.setDefaultCommand(Commands.run(() -> {
      ArmPosition pos = ArmPosition.getDefaultForPiece(rollers.getHeldPiece());
        arm.goToLocationCommand(pos);
      }
    ));

    ledControl.setDefaultCommand(ledControl.playPatternCommand(LEDs.m_scrollingRainbow));

    /* Operator bindings */

    //elevator height commands
    operatorController.L1().whileTrue(elevator.joystickControlCommand()).whileTrue(arm.joystickControlCommand());
    operatorController.triangle().whileTrue(Commands.run(() -> elevator.setHeight(1.7), elevator)); //L1 //66.93 inches
    // operatorController.circle().onTrue(Commands.run(() -> elevator.setHeightRegular(0.5))); //L2
    // operatorController.cross().onTrue(Commands.run(() -> elevator.setHeightRegular(0.75))); //L3
    // operatorController.square().onTrue(Commands.run(() -> elevator.setHeightRegular(1))); //4
    
    //roller intake/outtake commands
    // operatorController.R1().onTrue(rollers.intakeAlgaeCommand());
    // operatorController.R2().onTrue(rollers.outtakeAlgaeCommand());
    // operatorController.L1().onTrue(rollers.intakeCoralCommand());
    // operatorController.L2().onTrue(rollers.outtakeCoralCommand());

    // operatorController.triangle().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    // operatorController.circle().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // operatorController.cross().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    // operatorController.square().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));

    operatorController.circle().whileTrue(Commands.sequence(
      Commands.runOnce(() -> SignalLogger.start()),
      elevator.sysIdDynamic(Direction.kForward).until(elevator::isAtTop),
      elevator.sysIdDynamic(Direction.kReverse).until(elevator::isAtBottom),
      elevator.sysIdQuasistatic(Direction.kForward).until(elevator::isAtTop),
      elevator.sysIdQuasistatic(Direction.kReverse).until(elevator::isAtBottom),
      Commands.runOnce(() -> SignalLogger.stop())
    ).onlyWhile(() -> {
      return elevator.isSysIDSafe();
    }));

    operatorController.touchpad().onTrue(Commands.runOnce(() -> elevator.resetEncoders(0)));

    //SignalLogger.setPath("/media/sda1/");
    // operatorController.L2().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    // operatorController.R2().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

    operatorController.cross().onTrue(Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true)).onFalse(Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));

    /* DRIVER BINDINGS */

    //Robot relative
    driverController.L2()
      .onTrue(drivetrain.toRobotRelativeCommand())
      .onFalse(drivetrain.toFieldRelativeCommand());
    //90 degree buttons
    driverController.triangle()
      .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(0), false));
    driverController.square()
      .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(90), false));
    driverController.cross()
      .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(180), false));
    driverController.circle()
      .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(270), false)); 
    //zero gyro
    driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());
    //for testing robot relative angles
    driverController.L1().onTrue(drivetrain.alignToAngleRobotRelativeCommand(Rotation2d.fromDegrees(30), false));
    driverController.R1().onTrue(drivetrain.alignToAngleRobotRelativeCommand(Rotation2d.fromDegrees(-30), false));

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
    return auto.getAutoCommand();
  }
}

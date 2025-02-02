// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.ElementType;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.leds.BottomLEDs;
import frc.robot.leds.LEDs;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.vision.Limelight;


public class RobotContainer {

  //Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static CommandPS5Controller testingController = new CommandPS5Controller(2);

  public static final boolean debugMode = true;

  //Subsystems
  public static Drivetrain drivetrain = new Drivetrain();
  // public static Elevator elevator = new Elevator();
  public static Limelight limelight = new Limelight("Camera");
  public static BottomLEDs BottomLEDs = new BottomLEDs();
  public static Rollers rollers = new Rollers();

  public static Auto auto = new Auto();

  //Alliance
  public static boolean isBlueAlliance;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();

  }

  private void configureBindings() {
    
    /* Default commands */
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());
   // elevator.setDefaultCommand(elevator.joystickControlCommand());
    BottomLEDs.setDefaultCommand(BottomLEDs.playPatternCommand(LEDs.m_scrollingRainbow));

    /* Operator bindings */
    //elevator height commands
    // operatorController.triangle().onTrue(elevator.setHeightCommand(.33)); //L1
    // operatorController.circle().onTrue(elevator.setHeightCommand(.81)); //L2
    // operatorController.cross().onTrue(elevator.setHeightCommand(1.21)); //L3
    // operatorController.square().onTrue(elevator.setHeightCommand(1.83)); //4
    
    //roller intake/outtake commands
    operatorController.create().onTrue(rollers.switchSpeedCommand());
    operatorController.R1().onTrue(rollers.setIntakeCommand(1));
    operatorController.L1().onTrue(rollers.setOuttakeCommand(1));

    operatorController.triangle().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    operatorController.circle().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    operatorController.cross().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    operatorController.square().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

    //SignalLogger.setPath("/media/sda1/");
    operatorController.L2().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    operatorController.R2().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

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

  }

  public Command getAutonomousCommand() {
    return auto.getAutoCommand();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.annotation.ElementType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.leds.BottomLEDs;
import frc.robot.leds.LEDs;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator3;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.TestSubSystem;
import frc.robot.util.TunerConstants;
import frc.robot.vision.Limelight;


public class RobotContainer {

  //Controllers
  // public static CommandGenericHID driverController = new CommandGenericHID(0);
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static CommandPS5Controller testingController = new CommandPS5Controller(2);

  public static final boolean debugMode = true;

  //Subsystems
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator3 elevator = new Elevator3();
  public static Limelight limelight = new Limelight("Camera");
  public static BottomLEDs BottomLEDs = new BottomLEDs();
  public static Rollers rollers = new Rollers();
  // public static TestSubSystem testSubSystem = new TestSubSystem();

  public static Auto auto = new Auto();

  // Logging
  private final Telemetry telemetryLogger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  //Alliance
  public static boolean isBlueAlliance;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();

    // Reset robot to starting position
    drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
  }

  private void configureBindings() {

    
    /* Default commands */
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());
    // elevator.setDefaultCommand();
    driverController.button(2).onTrue(elevator.setHeightCommand(0));
    driverController.button(3).onTrue(elevator.setHeightCommand(Elevator3.ELEVATOR_MAX_HEIGHT/4));
    driverController.button(4).onTrue(elevator.setHeightCommand(Elevator3.ELEVATOR_MAX_HEIGHT/2));
    driverController.button(5).onTrue(elevator.setHeightCommand(3*Elevator3.ELEVATOR_MAX_HEIGHT/4));
    driverController.button(6).onTrue(elevator.setHeightCommand(Elevator3.ELEVATOR_MAX_HEIGHT));

    driverController.button(1).whileTrue(elevator.joystickControlCommand()).onFalse(elevator.stopCommand());

    // BottomLEDs.setDefaultCommand(BottomLEDs.playPatternCommand(LEDs.m_scrollingRainbow));

    /* Operator bindings */
    //elevator height commands
    // 
    // driverController.button(1).onTrue(Commands.runOnce(() -> System.out.println("b1")));
    // driverController.button(2).onTrue(Commands.runOnce(() -> System.out.println("b2")));
    // driverController.button(3).onTrue(Commands.runOnce(() -> System.out.println("b3")));
    // driverController.button(4).onTrue(Commands.runOnce(() -> System.out.println("b4")));
    // driverController.button(5).onTrue(Commands.runOnce(() -> System.out.println("b5")));
    // driverController.button(6).onTrue(Commands.runOnce(() -> System.out.println("b6")));
    // driverController.button(7).onTrue(Commands.runOnce(() -> System.out.println("b7")));
    // driverController.button(8).onTrue(Commands.runOnce(() -> System.out.println("b8")));
    // driverController.L1().onTrue(Commands.runOnce(() -> System.out.println("L1")));
    // driverController.L2().onTrue(Commands.runOnce(() -> System.out.println("L2")));
    // driverController.L3().onTrue(Commands.runOnce(() -> System.out.println("L3")));
    // driverController.R1().onTrue(Commands.runOnce(() -> System.out.println("R1")));
    // driverController.R2().onTrue(Commands.runOnce(() -> System.out.println("R2")));
    // driverController.R3().onTrue(Commands.runOnce(() -> System.out.println("R3")));
    // driverController.circle().onTrue(Commands.runOnce(() -> System.out.println("circle")));
    // driverController.cross().onTrue(Commands.runOnce(() -> System.out.println("cross")));
    // driverController.square().onTrue(Commands.runOnce(() -> System.out.println("square")));
    // driverController.triangle().onTrue(Commands.runOnce(() -> System.out.println("triangle")));
    // driverController.touchpad().onTrue(Commands.runOnce(() -> System.out.println("touchpad")));



    // operatorController.triangle().onTrue(elevator.setHeightCommand(.33)); //L1
    // operatorController.circle().onTrue(elevator.setHeightCommand(.81)); //L2
    // operatorController.cross().onTrue(elevator.setHeightCommand(1.21)); //L3
    // operatorController.square().onTrue(elevator.setHeightCommand(1.83)); //4
    
    //roller intake/outtake commands
    operatorController.create().onTrue(rollers.switchSpeedCommand());
    operatorController.R1().onTrue(rollers.setIntakeCommand(1));
    operatorController.L1().onTrue(rollers.setOuttakeCommand(1));


    /* DRIVER BINDINGS */

    //Robot relative
    // driverController.L2()
    //   .onTrue(drivetrain.toRobotRelativeCommand())
    //   .onFalse(drivetrain.toFieldRelativeCommand());
    // //90 degree buttons
    // driverController.triangle()
    //   .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(0), false));
    // driverController.square()
    //   .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(90), false));
    // driverController.cross()
    //   .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(180), false));
    // driverController.circle()
    //   .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(270), false)); 
    // //zero gyro
    // driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());
    // //for testing robot relative angles
    // driverController.L1().onTrue(drivetrain.alignToAngleRobotRelativeCommand(Rotation2d.fromDegrees(30), false));
    // driverController.R1().onTrue(drivetrain.alignToAngleRobotRelativeCommand(Rotation2d.fromDegrees(-30), false));

    drivetrain.registerTelemetry(telemetryLogger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return auto.getAutoCommand();
  }
}

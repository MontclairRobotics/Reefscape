// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.ElementType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.leds.BottomLEDs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.vision.Limelight;


public class RobotContainer {

  //Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static CommandPS5Controller testingController = new CommandPS5Controller(2);

  //Subsystems
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();
  public static Limelight limelight = new Limelight("Camera");
  public static BottomLEDs leds = new BottomLEDs();

  //Alliance
  public static boolean isBlueAlliance;

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());
    elevator.setDefaultCommand(elevator.joystickControlCommand());

    operatorController.triangle().onTrue(elevator.setHeightCommand(.33)); //L1
    operatorController.circle().onTrue(elevator.setHeightCommand(.81)); //L2
    operatorController.cross().onTrue(elevator.setHeightCommand(1.21)); //L3
    operatorController.square().onTrue(elevator.setHeightCommand(1.83)); //4
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  //Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public CommandPS5Controller testingController = new CommandPS5Controller(2);

  //Subsystems
  public Drivetrain drivetrain = new Drivetrain();

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInput());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

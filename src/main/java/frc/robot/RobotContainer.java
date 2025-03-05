// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.leds.LEDs;
import frc.robot.subsystems.Ratchet;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.util.RobotState;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.GamePiece;
import frc.robot.util.PoseUtils;
import frc.robot.util.TagOffset;
import frc.robot.util.TunerConstants;
import frc.robot.vision.ElevatorLimelight;
import frc.robot.vision.Limelight;


public class RobotContainer {

  //Controllers
  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  public static CommandPS5Controller testingController = new CommandPS5Controller(2);

  public static final boolean debugMode = false;
  public static final boolean logMode = false;

  //Subsystems
  public static Limelight leftLimelight = new Limelight("limelight-left", 0.38, 0, 0, 0, true);
  public static Limelight rightLimelight = new Limelight("limelight-right", 0.38, 0, 0, 0, false);
  public static Ratchet ratchet = new Ratchet();
  public static Drivetrain drivetrain = new Drivetrain();
  public static Elevator elevator = new Elevator();
  public static ElevatorLimelight elevatorLimelight = new ElevatorLimelight("limelight-elevator", 0, 0, 0, 0, true);
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
    //  configureDriveTryoutBindings();
    configureCompetitionBindings();
    //configureBindings();
    // Enables limelights when tethered over USB

    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
    // http://roborio-555-FRC.local:5801 will now forward to limelight-left.local:5801
    // http://roborio-555-FRC.local:5811 will now forward to limelight-right.local:5801
    for (int i = 5800; i <= 5807; i++) {
      PortForwarder.add(i, "limelight-left.local", i);
      PortForwarder.add(i+10, "limelight-right.local", i+10);
    }
  }

  private void configureCompetitionBindings() {

    /* --------------------------------------------OPERATOR BINDINGS --------------------------------------------*/

    rollers.setDefaultCommand(rollers.getDefaultCommand());

    
    //Intake
    operatorController.L1()
    .whileTrue(
      rollers.intakeCoralJiggleCommand()
      .alongWith(arm.holdState(RobotState.Intake))
      .alongWith(elevator.setState(RobotState.Intake)))
    .onFalse(
      rollers.stopCommand()
      .alongWith(arm.stopCommand())
      .alongWith(elevator.stopCommand())
    );

    //Scoring
    operatorController.R1()
      .onTrue(rollers.outtakeCoralCommand())
      .onFalse(rollers.stopCommand());

    //L1 scoring
    operatorController.R2().and(operatorController.cross())
      .whileTrue(rollers.scoreL1())
      .onFalse(rollers.stopCommand());

    // L1 
    operatorController.cross().and(operatorController.L2().negate())
      .whileTrue(arm.holdState(RobotState.L1))
      .onFalse(
        elevator.setState(RobotState.L1)
        .alongWith(arm.holdState(RobotState.L1))
      );
      
    // L2 
    operatorController.square().and(operatorController.L2().negate())
      .whileTrue(arm.holdState(RobotState.L2))
      .onFalse(
        elevator.setState(RobotState.L2)
        .alongWith(arm.holdState(RobotState.L2))
      );

    // L3
    operatorController.triangle().and(operatorController.L2().negate())
      .whileTrue((arm.holdState(RobotState.L3)))
      .onFalse(
        elevator.setState(RobotState.L3)
        .alongWith(arm.holdState(RobotState.L3))
      );

    // L4 
    operatorController.circle().and(operatorController.L2().negate())
      .whileTrue(arm.holdState(RobotState.L4))
      .onFalse(
        elevator.setState(RobotState.L4)
        .alongWith(arm.holdState(RobotState.L4))
      );

    // Elevator down
    operatorController.R2()
      .onTrue(elevator.setState(RobotState.DrivingNone))
      .onTrue(arm.holdState(RobotState.Intake));

    //Lower algae
    operatorController.cross().and(operatorController.L2())
      .whileTrue(
        arm.setState(RobotState.L1Algae)
        .alongWith(elevator.setState(RobotState.L1Algae))
        .alongWith(rollers.intakeAlgaeCommand())
      );

    //Higher algae
    operatorController.triangle().and(operatorController.L2())
      .whileTrue(
        arm.setState(RobotState.L2Algae)
        .alongWith(elevator.setState(RobotState.L2Algae))
        .alongWith(rollers.intakeAlgaeCommand())
      );

    //Climb
    operatorController.circle().and(operatorController.L2())
      .whileTrue(ratchet.engageServos())
      .onFalse(ratchet.disengageServos());
    
    // //Climb
    // operatorController.circle().and(operatorController.L2())
    //   .whileTrue(elevator.climbUpCommand())
    //   .onFalse(elevator.climbDownCommand());

    //Processor
    operatorController.square().and(operatorController.L2())
      .onTrue(arm.setState(RobotState.Processor));

    /*--------------------------------- DRIVER BINDINGS -------------------------------------------- */ 

    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());

    //alignment buttons
    driverController.R2()
      .whileTrue(new GoToPoseCommand(TagOffset.CENTER, true))
      .onFalse(new GoToPoseCommand(TagOffset.CENTER, false).until(() -> drivetrain.joystickInputDetected()));
    
    driverController.L1()
      .whileTrue(new GoToPoseCommand(TagOffset.LEFT, true))
      .onFalse(new GoToPoseCommand(TagOffset.LEFT, false).until(() -> drivetrain.joystickInputDetected()));
    
    driverController.R1()
      .whileTrue(new GoToPoseCommand(TagOffset.RIGHT, true))
      .onFalse(new GoToPoseCommand(TagOffset.RIGHT, false).until(() -> drivetrain.joystickInputDetected()));

    //Robot relative
    driverController.L2()
      .onTrue(drivetrain.toRobotRelativeCommand())
      .onFalse(drivetrain.toFieldRelativeCommand());

    // 90 degree buttons
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
    
    //telemetry
    drivetrain.registerTelemetry(telemetryLogger::telemeterize);    

    /* ---------------------------------------- TESTING BINDINGS --------------------------------------- */

    testingController.L1().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    testingController.R1().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
    testingController.triangle().whileTrue(
      drivetrain.sysIdDynamic(Direction.kForward)
    );
    testingController.circle().whileTrue(
      drivetrain.sysIdDynamic(Direction.kReverse)
    );
    testingController.cross().whileTrue(
      drivetrain.sysIdQuasistatic(Direction.kForward)
    );
    testingController.square().whileTrue(
      drivetrain.sysIdQuasistatic(Direction.kReverse)
    );

   }

  private void configureBindings() {

    // orchestra.addInstrument(elevator.leftTalonFX);
    // orchestra.addInstrument(elevator.rightTalonFX);
    // operatorController.R2().onTrue(playMusic("SevenNationArmy.chrp")).onFalse(stopMusic());

    /* Default commands */

    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());

    rollers.setDefaultCommand(rollers.getDefaultCommand());

    elevator.setDefaultCommand(Commands.run(() -> {
      RobotState pos = RobotState.getDefaultForPiece(rollers.getHeldPiece());
      // System.out.println(pos.getHeight());
      elevator.setExtension(pos.getHeight());
      }
    , elevator));

    arm.setDefaultCommand(Commands.run(() -> {
      //  RobotState pos = RobotState.getDefaultForPiece(rollers.getHeldPiece());
        arm.setEndpointAngle(
          //pos.getAngle()
          RobotState.Intake.getAngle()
          );
      }
    , arm));
    
    //elevator.setDefaultCommand(elevator.joystickControlCommand());
    //m.setDefaultCommand(arm.joystickControlCommand());
    // elevator.setDefaultCommand(elevator.joystickControlCommand());
    // leds.setDefaultCommand(elevator.isVelociatated() ? leds.playPatternCommand(LEDs.progress()) : rollers.getHeldPiece() == GamePiece.Algae ? leds.playPatternCommand(LEDs.holding(GamePiece.Algae.getColor())) : rollers.getHeldPiece() == GamePiece.Coral ? leds.playPatternCommand(LEDs.holding(GamePiece.Coral.getColor())) : leds.playPatternCommand(LEDs.breathingPattern()));
    // operatorController.square().whileTrue(arm.goToAngleCommand(Rotation2d.fromDegrees(30)));

    /* Operator bindings */

    //elevator height commands
    operatorController.L1().whileTrue(elevator.joystickControlCommand()).whileTrue(arm.joystickControlCommand()).onFalse(Commands.runOnce(() -> {
      elevator.stop();
      arm.stopMotor();
    }));
    // operatorController.triangle().whileTrue(Commands.run(() -> elevator.setHeight(1.7), elevator)); //L1 //66.93 inches
    

    // Intake coral
    // operatorController.L2()
    //   .whileTrue(
    //     elevator.setScoringHeightCommand(RobotState.Intake)
    //     .alongWith(arm.goToLocationCommand(RobotState.Intake))
    //     .alongWith(rollers.intakeCoralCommand()))
    //   .onFalse(rollers.stop());

    //Intake algae
    operatorController.R2().whileTrue(rollers.outtakeCoralCommand()).onFalse(rollers.stopCommand());

    operatorController.R1()
      .whileTrue(rollers.intakeAlgaeCommand())
      .onFalse(rollers.stopCommand());

    
    // //Arm coast mode
    // testingController.circle().onTrue(arm.setIdleModeCommand(IdleMode.kCoast)).onFalse(arm.setIdleModeCommand(IdleMode.kBrake));
    
    //Coral intake outtake
    // operatorController.L1()
    //   .whileTrue(rollers.intakeCoralCommand())
    //   .onFalse(rollers.stopCommand());
    //Intaking
    operatorController.L2().whileTrue(rollers.intakeCoralJiggleCommand().alongWith(arm.holdState(RobotState.Intake)))
    .onFalse(rollers.stopCommand().alongWith(arm.stopCommand()));

    //Ratchet Bindings
    operatorController.povLeft()
      .onTrue(ratchet.engageServos());
    operatorController.povRight()
      .onTrue(ratchet.engageServos());
    /* SETS DIFFERENT ROBOT STATES */
  
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

    // testingController.triangle().whileTrue(Commands.sequence(
    //   Commands.runOnce(() -> SignalLogger.start()),
    //   drivetrain.sysIdDynamic(Direction.kForward),
    //   drivetrain.sysIdDynamic(Direction.kReverse),
    //   drivetrain.sysIdQuasistatic(Direction.kForward),
    //   drivetrain.sysIdQuasistatic(Direction.kReverse),
    //   Commands.runOnce(() -> SignalLogger.stop())
    // ));

    //resets elevator encoders
    operatorController.touchpad().onTrue(
      Commands.runOnce(() -> elevator.resetEncoders(0))
      .ignoringDisable(true)
    );

    // //Coast mode elevator
    // testingController.cross()
    //   .onTrue(
    //     Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Coast))
    //     .ignoringDisable(true)
    //   )
    //   .onFalse(
    //     Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Brake))
    //     .ignoringDisable(true)
    //   );

    /* DRIVER BINDINGS */

    //alignment buttons
    driverController.R2().whileTrue(new GoToPoseCommand(TagOffset.CENTER, true)
     // .andThen(new AlignToAprilTagCommand(ScoreDirection.CENTER))
     );
    driverController.L1().whileTrue(new GoToPoseCommand(TagOffset.LEFT, true)
    //  .andThen(new AlignToAprilTagCommand(ScoreDirection.LEFT))
    );
    driverController.R1().whileTrue(new GoToPoseCommand(TagOffset.RIGHT, true)
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

    //Robot relative
    driverController.L2()
      .onTrue(drivetrain.toRobotRelativeCommand())
      .onFalse(drivetrain.toFieldRelativeCommand());

    /* Operator bindings */

    // //Arm coast mode
    // testingController.circle().onTrue(arm.setIdleModeCommand(IdleMode.kCoast)).onFalse(arm.setIdleModeCommand(IdleMode.kBrake));

    // //Coast mode elevator
    // testingController.cross()
    //   .onTrue(
    //     Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Coast))
    //     .ignoringDisable(true)
    //   )
    //   .onFalse(
    //     Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Brake))
    //     .ignoringDisable(true)
    //   );

    //resets elevator encoders
    operatorController.touchpad().onTrue(
      Commands.runOnce(() -> elevator.resetEncoders(0))
      .ignoringDisable(true)
    );

    //Allow Manuel Control
    operatorController.L1().whileTrue(elevator.joystickControlCommand()).whileTrue(arm.joystickControlCommand()).onFalse(Commands.runOnce(() -> {
      elevator.stop();
      arm.stopMotor();
    }));    

    //Intake algae
    operatorController.R1()
      .whileTrue(rollers.intakeAlgaeCommand())
      .onFalse(rollers.stopCommand());
    //Outtake algae
    operatorController.R2()
      .whileTrue(rollers.outtakeCoralCommand())
      .onFalse(rollers.stopCommand());

    operatorController.R2().and(operatorController.cross())
      .whileTrue(rollers.scoreL1())
      .onFalse(rollers.stopCommand());
    //Ratchet Bindings
    operatorController.povLeft()
      .onTrue(ratchet.engageServos());

    operatorController.povRight()
      .onTrue(ratchet.engageServos());

    /* SETS DIFFERENT ROBOT STATES */

    // operatorController.L1().whileTrue(arm.setState(RobotState.Intake).alongWith(rollers.intakeCoralCommand())) //intake state
    // .onFalse(arm.stopCommand().alongWith(rollers.stopCommand()));

    // operatorController.triangle().whileTrue(arm.setState(RobotState.L4)); 
    // operatorController.square().whileTrue(arm.setState(RobotState.Intake));

    /* Reef heights */

    // L1
    operatorController.cross()
      .whileTrue(arm.setState(RobotState.L1)
      .alongWith(elevator.setState(RobotState.L1)))
      .onFalse(arm.stopCommand()
      .alongWith(elevator.stopCommand()));
    // L2
    operatorController.square()
      .whileTrue(arm.setState(RobotState.L2)
      .alongWith(elevator.setState(RobotState.L2)))
      .onFalse(arm.stopCommand()
      .alongWith(elevator.stopCommand()));
    // L3
    operatorController.triangle()
      .whileTrue((arm.setState(RobotState.L3))
      .alongWith(elevator.setState(RobotState.L3)))
      .onFalse((arm.stopCommand())
      .alongWith(elevator.stopCommand()));
    // L4
    operatorController.circle()
    .whileTrue(arm.setState(RobotState.L4).alongWith(elevator.setState(RobotState.L4)))
    .onFalse(arm.stopCommand().alongWith(elevator.stopCommand()));
  
    /* sysId */

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

    // testingController.triangle().whileTrue(Commands.sequence(
    //   Commands.runOnce(() -> SignalLogger.start()),
    //   drivetrain.sysIdDynamic(Direction.kForward),
    //   drivetrain.sysIdDynamic(Direction.kReverse),
    //   drivetrain.sysIdQuasistatic(Direction.kForward),
    //   drivetrain.sysIdQuasistatic(Direction.kReverse),
    //   Commands.runOnce(() -> SignalLogger.stop())
    // ));

    testingController.L1().onTrue(Commands.runOnce(() -> SignalLogger.start()));
    testingController.R1().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
    testingController.triangle().whileTrue(
      drivetrain.sysIdDynamic(Direction.kForward)
    );
    testingController.circle().whileTrue(
      drivetrain.sysIdDynamic(Direction.kReverse)
    );
    testingController.cross().whileTrue(
      drivetrain.sysIdQuasistatic(Direction.kForward)
    );
    testingController.square().whileTrue(
      drivetrain.sysIdQuasistatic(Direction.kReverse)
    );
  }

  // public void configureDriveTryoutBindings() {

  //   /* DRIVER BINDINGS */
  //   drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());

  //   //zeros gyro
  //   driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());
  //   //robot relative
  //   driverController.L2().onTrue(drivetrain.toRobotRelativeCommand()).onFalse(drivetrain.toFieldRelativeCommand());

  //   /* 90 degree buttons */
    
  //   //0 degrees
  //   driverController.triangle().and(driverController.R1().negate())
  //     .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
  //   //90 degrees
  //   driverController.square().and(driverController.R1().negate())
  //     .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(90)), false));
  //   //180 degrees
  //     driverController.cross().and(driverController.R1().negate())
  //     .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
  //   //270 degrees
  //     driverController.circle().and(driverController.R1().negate())
  //     .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(270)), false)); 


  //   driverController.triangle().and(driverController.R1())
  //     .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(-60)), false));
  //   driverController.square().and(driverController.R1())
  //     .onTrue(RobotContainer.drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(60)), false));
  //   driverController.cross().and(driverController.R1())
  //     .onTrue(drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(120)), false));
  //   driverController.circle().and(driverController.R1())
  //     .onTrue(RobotContainer.drivetrain.alignToAngleFieldRelativeCommand(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(-120)), false));
  //   /* OPERATOR BINDINGS */

  //   arm.setDefaultCommand(Commands.run(() -> {
  //     //RobotState pos = RobotState.getDefaultForPiece(rollers.getHeldPiece());
  //     //arm.setEndpointAngle(pos.getAngle());
  //     arm.setState(RobotState.Intake);
  //   }, arm));

  //   elevator.setDefaultCommand(
  //     Commands.run(() -> {
  //       //RobotState pos = RobotState.getDefaultForPiece(rollers.getHeldPiece());
  //       // elevator.setExtension(pos.getHeight());

  //      elevator.setExtension(0);
  //     }, elevator)
  //   );

  //   //L1
  //   operatorController.cross()
  //     .whileTrue(arm.setState(RobotState.L1).alongWith(elevator.setState(RobotState.L1)))
  //     .onFalse(arm.stopCommand().alongWith(elevator.stopCommand()));
  //   //L2
  //   operatorController.square()
  //     .whileTrue(arm.setState(RobotState.L2).alongWith(elevator.setState(RobotState.L2)))
  //     .onFalse(arm.stopCommand().alongWith(elevator.stopCommand()));
  //   //L3
  //   operatorController.triangle()
  //     .whileTrue((arm.setState(RobotState.L3)).alongWith(elevator.setState(RobotState.L3)))
  //     .onFalse((arm.stopCommand()).alongWith(elevator.stopCommand()));
  //   //L4
  //   operatorController.circle()
  //     .whileTrue(arm.setState(RobotState.L4).alongWith(elevator.setState(RobotState.L4)))
  //     .onFalse(arm.stopCommand().alongWith(elevator.stopCommand()));

  
  //   //Intaking
  //   operatorController.L2().whileTrue(rollers.intakeCoralJiggleCommand().alongWith(arm.setState(RobotState.Intake)))
  //   .onFalse(rollers.stopCommand().alongWith(arm.stopCommand()));
  //   //scoring
  //   operatorController.R2().whileTrue(rollers.outtakeCoralCommand()).onFalse(rollers.stopCommand());
  //   //enables joystick control
  //   //operatorController.L1()/*.whileTrue(arm.joystickControlCommand())*/.whileTrue(elevator.joystickControlCommand());


  // }

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
    // System.out.println(auto.getAutoCommand());
    return auto.getAutoCommand();
  }
}
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BreakBeam;

public class Rollers extends SubsystemBase {
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    private final double CORAL_SPEED = 1;
    private final double ALGAE_SPEED = 0.5;
    private BreakBeam breakBeam = new BreakBeam(0, false);

    public Rollers() {
        rightMotor = new SparkMax(0, MotorType.kBrushless);
        leftMotor = new SparkMax(0, MotorType.kBrushless);
    }

    private boolean hasObject () {
        return breakBeam.get();
    }

    private void intakeAlgae () {
        rightMotor.set (ALGAE_SPEED);
        leftMotor.set (ALGAE_SPEED);
    }

    private void outtakeAlgae (){
        rightMotor.set(-ALGAE_SPEED);
        leftMotor.set(-ALGAE_SPEED);
    }

    private void intakeCoral (){
        rightMotor.set(CORAL_SPEED);
        leftMotor.set(CORAL_SPEED);
    }
  
    private void outtakeCoral (){
        rightMotor.set(-CORAL_SPEED);
        leftMotor.set(-CORAL_SPEED);
    }

    private void stopMotors (){
        rightMotor.set (0);
        leftMotor.set (0);
    }
    public Command intakeAlgaeCommand() {
        return Commands.run (() -> intakeAlgae(), this)
        .until (() -> hasObject())
        .finallyDo (() -> stopMotors());
    }

    public Command outtakeAlgaeCommand () {
        return Commands.runOnce(() -> outtakeAlgae(), this)
        .onlyWhile (() -> hasObject())
        .finallyDo (() -> stopMotors());
    }

    public Command intakeCoralCommand () {
        return Commands.runOnce(() -> intakeCoral(), this)
        .onlyWhile (() -> hasObject())
        .finallyDo (() -> stopMotors());
    }

    public Command outtakeCoralCommand () {
        return Commands.runOnce(() -> outtakeCoral(), this)
        .onlyWhile (() -> hasObject())
        .finallyDo (() -> stopMotors());
    }
}

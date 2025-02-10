package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BreakBeam;

public class Rollers extends SubsystemBase {
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    public final double CORAL_INTAKE_SPEED = 1;
    public final double CORAL_OUTTAKE_SPEED = -1;
    public final double ALGAE_INTAKE_SPEED = 0.5;
    public final double ALGAE_OUTTAKE_SPEED = -0.5;
    private BreakBeam breakBeam = new BreakBeam(0, false);

    public Rollers() {
        rightMotor = new SparkMax(13, MotorType.kBrushless);
        leftMotor = new SparkMax(14, MotorType.kBrushless);
    }

    public boolean hasObject() {
        return breakBeam.get();
    }

    private void setSpeed(double speed) {
        setSpeed(speed, speed);
    }

    private void setSpeed(double leftSpeed, double rightSpeed) {
        rightMotor.set(leftSpeed);
        leftMotor.set(rightSpeed);
    }

    private void stopMotors() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public Command autoShoot() {
        return Commands.run(() -> setSpeed(ALGAE_OUTTAKE_SPEED))
        .withTimeout(0.2);
    }
    
    public Command intakeAlgaeCommand() {
        return Commands.run(() -> setSpeed(ALGAE_INTAKE_SPEED), this)
                .until(() -> hasObject())
                .finallyDo(() -> stopMotors());
    }

    public Command outtakeAlgaeCommand() {
        return Commands.run(() -> setSpeed(ALGAE_OUTTAKE_SPEED), this)
                .onlyWhile(() -> hasObject())
                .finallyDo(() -> stopMotors());
    }

    public Command intakeCoralCommand() {
        return Commands.run(() -> setSpeed(CORAL_INTAKE_SPEED), this)
                .until(() -> hasObject())
                .finallyDo(() -> stopMotors());
    }

    public Command outtakeCoralCommand() {
        return Commands.run(() -> setSpeed(CORAL_OUTTAKE_SPEED), this)
                .onlyWhile(() -> hasObject())
                .finallyDo(() -> stopMotors());
    }
}

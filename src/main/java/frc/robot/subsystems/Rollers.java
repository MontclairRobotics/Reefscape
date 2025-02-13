package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Rollers extends SubsystemBase {
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    public final double CORAL_INTAKE_SPEED = 1;
    public final double CORAL_OUTTAKE_SPEED = -1;
    public final double ALGAE_INTAKE_SPEED = 0.5;
    public final double ALGAE_OUTTAKE_SPEED = -0.5;
    public final double ROLLER_STALL_CURRENT = 40; // TODO check/tune

    private GamePiece heldPiece = GamePiece.None; // TODO init to Coral for auton? not needed?

    public Rollers() {
        rightMotor = new SparkMax(13, MotorType.kBrushless);
        leftMotor = new SparkMax(14, MotorType.kBrushless);
    }

    public GamePiece getHeldPiece() {
        return heldPiece;
    }

    // TODO need to be debounced? probably not?
    public boolean isStalled() {
        return rightMotor.getOutputCurrent() > ROLLER_STALL_CURRENT
                || leftMotor.getOutputCurrent() > ROLLER_STALL_CURRENT;
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
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.Algae;
                }).until(this::isStalled);
    }

    public Command outtakeAlgaeCommand() {
        return Commands.run(() -> setSpeed(ALGAE_OUTTAKE_SPEED), this)
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.None;
                }).withTimeout(2); // TODO find timeout
    }

    public Command intakeCoralCommand() {
        return Commands.run(() -> setSpeed(CORAL_INTAKE_SPEED), this)
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.Coral;
                }).until(this::isStalled);
    }

    public Command outtakeCoralCommand() {
        return Commands.run(() -> setSpeed(CORAL_OUTTAKE_SPEED), this)
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.None;
                }).withTimeout(2); // TODO find timeout
    }
}

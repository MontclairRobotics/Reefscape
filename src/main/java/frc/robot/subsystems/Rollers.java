package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Rollers extends SubsystemBase {
    public SparkMax rightMotor;
    public SparkMax leftMotor;
    public final double CORAL_INTAKE_SPEED = 0.5;
    public final double CORAL_OUTTAKE_SPEED = -1;
    public final double ALGAE_INTAKE_SPEED = 0.2;
    public final double ALGAE_OUTTAKE_SPEED = -0.5;
    public final double ROLLER_STALL_CURRENT = 43; // TODO check/tune
    

    private GamePiece heldPiece = GamePiece.None; // TODO init to Coral for auton? not needed?

    public Rollers() {
        rightMotor = new SparkMax(31, MotorType.kBrushless);
        leftMotor = new SparkMax(30, MotorType.kBrushless);

        var config = new SparkMaxConfig();
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        rightMotor.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public GamePiece getHeldPiece() {
        return heldPiece;
    }

    // TODO need to be debounced? probably not?
    public boolean isStalled() {
        return rightMotor.getOutputCurrent() > ROLLER_STALL_CURRENT
                || leftMotor.getOutputCurrent() > ROLLER_STALL_CURRENT;
    }

    public void setSpeed(double speed) {
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

    public Command stop() {
        return Commands.runOnce(() -> stopMotors());
    }

    public Command intakeAlgaeCommand() {
        return Commands.run(() -> setSpeed(ALGAE_INTAKE_SPEED), this)
                .finallyDo(() -> {
                    setSpeed(0);
                    this.heldPiece = GamePiece.Algae;
                });
                // .until(this::isStalled);
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
                })
                
                
                 .until(this::isStalled);
                // .andThen(
                //     () -> setSpeed(CORAL_INTAKE_SPEED), this
                // ).withTimeout(0.2);
    }

    public Command outtakeCoralCommand() {
        return Commands.run(() -> setSpeed(CORAL_OUTTAKE_SPEED), this)
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.None;
                }).withTimeout(2); // TODO find timeout
    }

    public void periodic() {
        SmartDashboard.putNumber("Right Motor Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Left Motor Current", leftMotor.getOutputCurrent());
    }
}

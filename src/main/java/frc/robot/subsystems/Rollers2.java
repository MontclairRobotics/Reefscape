package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers2 extends SubsystemBase {

    public static final double ROLLER_STALL_CURRENT = 40;

    public static final double CORAL_INTAKE_SPEED = 0.5;
    public static final double CORAL_OUTTAKE_SPEED = 0.5;
    public static final double ALGAE_INTAKE_SPEED = 0.5;
    public static final double ALGAE_OUTTAKE_SPEED = 0.5;

    private SparkMax rightMotor;
    private SparkMax leftMotor;

    public Rollers2() {
        rightMotor = new SparkMax(30, MotorType.kBrushless);
        leftMotor = new SparkMax(13, MotorType.kBrushless);
    }

    public void intake(double speed) {
        rightMotor.set(speed);
        leftMotor.set(speed);
    }
    public void outtake(double speed) {
        rightMotor.set(-speed);
        leftMotor.set(-speed);
    }
    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }
    public boolean isStalled() {
        return rightMotor.getOutputCurrent() > ROLLER_STALL_CURRENT || leftMotor.getOutputCurrent() > ROLLER_STALL_CURRENT;
    }

    // TODO: Should these have timeouts?
    public Command intakeCoralCommand() {
        return Commands.run(() -> intake(CORAL_INTAKE_SPEED), this).until(() -> isStalled()).andThen(() -> stop());
    }
    public Command outakeCoralCommand() {
        return Commands.run(() -> intake(CORAL_OUTTAKE_SPEED), this).andThen(() -> stop());
    }

    public Command intakeAlgaeCommand() {
        return Commands.run(() -> intake(ALGAE_INTAKE_SPEED), this).until(() -> isStalled()).andThen(() -> stop());
    }
    public Command outakeAlgaeCommand() {
        return Commands.run(() -> intake(ALGAE_OUTTAKE_SPEED), this).andThen(() -> stop());
    }



    
}

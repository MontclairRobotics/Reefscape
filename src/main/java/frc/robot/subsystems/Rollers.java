package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    private double targetSpeed;

    public Rollers() {
    rightMotor = new SparkMax(0, MotorType.kBrushless);
    leftMotor = new SparkMax(0, MotorType.kBrushless);
    }
    public void setIntakeSpeed (double speed) {
        targetSpeed = speed;
    }

    public void setIntake () {
    rightMotor.set (targetSpeed);
    leftMotor.set (targetSpeed);
    }

    public void setOuttake (){
    rightMotor.set (-targetSpeed);
    leftMotor.set (-targetSpeed);
    }
    public Command setIntakeCommand(double targetSpeed) {
        return Commands.run(() -> setIntake(), this);
    }
    public Command setOuttakeCommand (double targetSpeed) {
        return Commands.run(() -> setOuttake(), this);
    }
    
}

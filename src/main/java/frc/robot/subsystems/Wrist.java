package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private SparkMax wristMotor;
    public double smallWristAngle;
    public double largeWristAngle;
    Encoder wristEncoder = (Encoder) wristMotor.getEncoder();
    public Wrist() {
        wristMotor = new SparkMax(13, MotorType.kBrushless);
    }

    private void setSpeed(double wristSpeed) {
        wristMotor.set(wristSpeed);
    }

    private void stopMotor() {
        wristMotor.stopMotor();
    }
    public void periodic(){
        largeWristAngle = wristEncoder.get();
        smallWristAngle = largeWristAngle * (30/14);
    }
}

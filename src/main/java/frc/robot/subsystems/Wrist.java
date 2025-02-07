package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Rollers;

public class Wrist extends SubsystemBase {
    private SparkMax wristMotor;
    public static Rollers rollers = new Rollers();
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
    private boolean atBranch = (wristEncoder.get() == 180);
    public Command placeCoral() {
        return Commands.run(() -> setSpeed(10), this)
                .until(() -> !atBranch)
                .andThen(() -> rollers.outtakeCoralCommand());
    }
    private boolean atSource = (wristEncoder.get() == 360);
    public Command getCoral() {
        return Commands.run(() -> setSpeed(10), this)
                .until(() -> !atSource)
                .finallyDo(() -> rollers.intakeCoralCommand());
    }
    public void periodic(){
        largeWristAngle = wristEncoder.get();
        smallWristAngle = largeWristAngle * (30/14);
        
    }
}

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
    private double altSpeed = targetSpeed/2;
// The second speed ↑
    public boolean doAltSpeed = false;
// If true, this ↑ sets the speed to be 1/2 of the target speed

    public Rollers() {
    //rightMotor = new SparkMax(0, MotorType.kBrushless);
    //leftMotor = new SparkMax(0, MotorType.kBrushless);
    }
    public void setIntakeSpeed (double speed) {
        targetSpeed = speed;
    }

    public void switchSpeed() {
     doAltSpeed = !doAltSpeed;
    }

    public void setIntake () {
    if(doAltSpeed){
        rightMotor.set (altSpeed);
        leftMotor.set (altSpeed);
    } else {
        rightMotor.set (targetSpeed);
        leftMotor.set (targetSpeed);
    }
    }
    public void setOuttake (){
        if(doAltSpeed){
            rightMotor.set (-altSpeed);
            leftMotor.set (-altSpeed);
        } else {
            rightMotor.set (-targetSpeed);
            leftMotor.set (-targetSpeed);
        }
    }
    public Command setIntakeCommand(double targetSpeed) {
        return Commands.run(() -> setIntake(), this);
    }
    public Command setOuttakeCommand (double targetSpeed) {
        return Commands.run(() -> setOuttake(), this);
    }
    public Command switchSpeedCommand () {
        return Commands.run(() -> switchSpeed());
    }
}

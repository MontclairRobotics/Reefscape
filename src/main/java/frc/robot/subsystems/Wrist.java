package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Wrist extends SubsystemBase {
    public final double WRIST_SPEED = 0.5;
    public final double MAX_VELOCITY = 0.5;
    public final double MAX_ACCELERATION = 0.5;
    private final ProfiledPIDController wristController;
    private Encoder encoder;
    private SparkMax wristMotor;
    public static Rollers rollers = new Rollers();
    public double smallWristAngle;
    public double largeWristAngle;
    public Wrist() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.0, 1.0);
        wristMotor = new SparkMax(13, MotorType.kBrushless);
        encoder = new Encoder(0,1);
        wristController = new ProfiledPIDController(0.5, 0.0, 0.1, constraints);
        encoder.setDistancePerPulse(1.0);
    }

    public void stopMotor() {
        wristMotor.stopMotor();
    }
    public void setWristTarget(double targetPosition) {
        wristController.setGoal(targetPosition);
    }
    public void updatePID() {
        double currentPosition = encoder.getDistance();
        double pidOutput = wristController.calculate(currentPosition);
        wristMotor.set(pidOutput); 
    }
    public void periodic(){
        largeWristAngle = encoder.getDistance();
        smallWristAngle = (largeWristAngle * (30.0/14.0)) + 30;
        updatePID();
        
    }
}

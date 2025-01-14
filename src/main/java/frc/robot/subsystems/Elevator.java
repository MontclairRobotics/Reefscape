package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase { 
    TalonFX leftElevatorTalonFX;
    TalonFX rightElevatorTalonFX;
    CANcoder elevCANcoder;
    PIDController pidController;
public Elevator(){
    TalonFX leftElevatorTalonFX = new TalonFX(0,"bus");
    TalonFX rightElevatorTalonFX = new TalonFX(0, "bus");
    CANcoder elevCANcoder = new CANcoder(0, "bus");
    PIDController pidController = new PIDController(3,4,1);
}
    ElevatorFeedforward feedforward = new ElevatorFeedforward(2,0,4,8);
    
    DigitalInput bottomLimit = new DigitalInput(0);
    DigitalInput topLimit = new DigitalInput(0);
    
    public boolean isAtBottom() {
        return bottomLimit.get(); 
    }
    public boolean isAtTop() {
        return topLimit.get(); 
    }

    public void elevatorUp() {
        rightElevatorTalonFX.set(ElevatorConstants.ELEVATOR_SPEED);
        leftElevatorTalonFX.set(ElevatorConstants.ELEVATOR_SPEED);
    }
    public void elevatorDown() { 
        leftElevatorTalonFX.set(-ElevatorConstants.ELEVATOR_SPEED);
        rightElevatorTalonFX.set(-ElevatorConstants.ELEVATOR_SPEED);   
    }
    public void ElevatorStop() {
        leftElevatorTalonFX.set(0);
        rightElevatorTalonFX.set(0);
    }
    // public boolean isInPosition() {
    //     return 
        
    // }
}

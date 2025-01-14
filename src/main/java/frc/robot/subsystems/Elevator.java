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
    TalonFX leftElevatorTalonFX = new TalonFX(0,"bus");
    TalonFX rightElevatorTalonFX = new TalonFX(0, "bus");
    PIDController pidController = new PIDController(5, 5, 5);
    CANcoder elevCANcoder = new CANcoder(0, "bus");
    
    ElevatorFeedforward feedforward = new ElevatorFeedforward(1,1,1,1);
    
    DigitalInput bottomLimit = new DigitalInput(0);
    DigitalInput topLimit = new DigitalInput(0);
    

    public void elevatorUp(){ // UP
        rightElevatorTalonFX.set(ElevatorConstants.ELEVATOR_SPEED);
        leftElevatorTalonFX.set(ElevatorConstants.ELEVATOR_SPEED);
    }
    public void elevatorDown(){ // DOWN
        leftElevatorTalonFX.set(-ElevatorConstants.ELEVATOR_SPEED);
        rightElevatorTalonFX.set(-ElevatorConstants.ELEVATOR_SPEED);   
    }
    public void ElevatorStop(){ //STOP
        leftElevatorTalonFX.set(0);
        rightElevatorTalonFX.set(0);
    }

}

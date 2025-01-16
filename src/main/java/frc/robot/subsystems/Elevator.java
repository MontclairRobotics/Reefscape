package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {
    //Constents
    private final double ENCODER_ROTATIONS_TO_METERS_RATIO = 0.5; //TODO: FIND THIS
    //Motor Controllers/Encoders
    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;
    private CANcoder canCoder;
    
    private PIDController pidController;
    private ElevatorFeedforward elevatorFeedforward;
    //Limit Switches
    private DigitalInput bottomLimit;
    private DigitalInput topLimit;

    public Elevator() {
        leftTalonFX = new TalonFX(-1, "rio");
        rightTalonFX = new TalonFX(-1, "rio");
        canCoder = new CANcoder(-1, "rio");

        pidController = new PIDController(0, 0, 0);
        elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0); //TODO: Turn

        bottomLimit = new DigitalInput(-1); //TODO: get
        topLimit = new DigitalInput(-1);
    }
        
    public boolean isAtBottom() {
        return bottomLimit.get(); 
    }

    public boolean isAtTop() {
        return topLimit.get(); 
    }
    //Manual Controll
    public void joystickControl() {
        double voltage = RobotContainer.operatorController.getLeftY() * 12 + elevatorFeedforward.calculate(0); //Multiplying by max voltage (12)
        if (isAtTop() && voltage > 0) { 
            stop();
        } else if (isAtBottom() && voltage < 0) {
            stop();
        } else {
            leftTalonFX.setVoltage(voltage);
            rightTalonFX.setVoltage(voltage);
        }
    }

    public void stop() {
        leftTalonFX.setVoltage(0);
        rightTalonFX.setVoltage(0);
    
    }
    //To Positions
    public void setHeight(double height) {
        double rotations = height * ENCODER_ROTATIONS_TO_METERS_RATIO;
        leftTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble(), rotations) + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble(), rotations) + elevatorFeedforward.calculate(0));
    }

    //Commands
    public Command joystickControlCommand() {
        return Commands.runOnce( ()-> joystickControl());
    }

    public Command stopCommand() {
        return Commands.runOnce(()-> stop());
    }

    public Command setHeightCommand(double height) {
        return Commands.runOnce(()-> setHeight(height));
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    //Constents
    private final double encoderRotationsToMetersRatio = 0.5;
    private final double heightL1 = 3; //Meters
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
        leftTalonFX = new TalonFX(0, "rio");
        rightTalonFX = new TalonFX(0, "rio");
        canCoder = new CANcoder(0, "rio");

        pidController = new PIDController(0, 0, 0);
        elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0);

        bottomLimit = new DigitalInput(-1);
        topLimit = new DigitalInput(-1);
    }
        
    public boolean isAtBottom() {
        return bottomLimit.get(); 
    }
    public boolean isAtTop() {
        return topLimit.get(); 
    }

    //Manual Controll
    public void up(CommandPS5Controller OperatorController) {
        if (isAtTop()) {
            stop();
        } else {
            leftTalonFX.setVoltage(OperatorController.getLeftY() * ElevatorConstants.ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
            rightTalonFX.setVoltage(OperatorController.getLeftY() * ElevatorConstants.ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
        }
    }

    public void down(CommandPS5Controller OperatorController) { 
        if (isAtBottom()) {
            stop();
        } else {
        leftTalonFX.setVoltage(OperatorController.getLeftY() * -ElevatorConstants.ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(OperatorController.getLeftY() * -ElevatorConstants.ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
        }
    }

    public void stop() {
        leftTalonFX.setVoltage(elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(elevatorFeedforward.calculate(0));
    
    }

    public void goToPositionL1(CommandPS5Controller OperatorController) {
        leftTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / encoderRotationsToMetersRatio, heightL1) + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / encoderRotationsToMetersRatio, heightL1) + elevatorFeedforward.calculate(0));

    }

    public void goToPositionL2() {
        
    }

    public void goToPositionL3() {
        
    }

    public void goToPositionL4() {
        
    }
}

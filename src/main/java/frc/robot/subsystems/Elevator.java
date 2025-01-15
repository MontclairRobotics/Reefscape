package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class Elevator extends SubsystemBase {
    //Constents
    private final double ENCODER_ROTATIONS_TO_METERS_RATIO = 0.5;
    private final double HEIGHT_L1 = 3; //Meters
    private final double HEIGHT_L2 = 4;
    private final double HEIGHT_L3 = 5;
    private final double HEIGHT_L4 = 6;
    private final double ELEVATOR_SPEED = 2; //m/s
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
    //Manual Controll TODO: THIS IS TOTTALLY WRONG BUT WE HAVE TO DECIDE IF WE WANT JOYSTICKS OR BUTTONS TO DO RIGHT
    public void up(CommandPS5Controller OperatorController) {
        if (isAtTop()) {
            stop();
        } else {
            leftTalonFX.setVoltage(OperatorController.getLeftY() * ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
            rightTalonFX.setVoltage(OperatorController.getLeftY() * ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
        }
    }

    public void down(CommandPS5Controller OperatorController) { 
        if (isAtBottom()) {
            stop();
        } else {
        leftTalonFX.setVoltage(OperatorController.getLeftY() * -ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(OperatorController.getLeftY() * -ELEVATOR_SPEED + elevatorFeedforward.calculate(0));
        }
    }

    public void stop() {
        leftTalonFX.setVoltage(elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(elevatorFeedforward.calculate(0));
    
    }
    //To Positions
    public void goToPositionL1() {
        leftTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L1) + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L1) + elevatorFeedforward.calculate(0));
    }

    public void goToPositionL2() {
        leftTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L2) + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L2) + elevatorFeedforward.calculate(0));
    }

    public void goToPositionL3() {
        leftTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L3) + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L3) + elevatorFeedforward.calculate(0));
    }

    public void goToPositionL4() {
        leftTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L4) + elevatorFeedforward.calculate(0));
        rightTalonFX.setVoltage(pidController.calculate(canCoder.getPosition().getValueAsDouble() / ENCODER_ROTATIONS_TO_METERS_RATIO, HEIGHT_L4) + elevatorFeedforward.calculate(0));
    }

    //Commands
    public Command upCommand(CommandPS5Controller OperatorController) {
        return Commands.runOnce( ()-> up(OperatorController));
    }

    public Command downCommand(CommandPS5Controller OperatorController) {
        return Commands.runOnce( ()-> down(OperatorController));
    }

    public Command stopCommand() {
        return Commands.runOnce( ()-> stop());
    }

    public Command goToPositionL1Command() { //This is run not run once is that right?
        return Commands.run(()-> goToPositionL1());
    }

    public Command goToPositionL2Command() {
        return Commands.run(()-> goToPositionL2());
    }

    public Command goToPositionL3Command() {
        return Commands.run(()-> goToPositionL3());
    }

    public Command goToPositionL4Command() {
        return Commands.run(()-> goToPositionL4());
    }
}

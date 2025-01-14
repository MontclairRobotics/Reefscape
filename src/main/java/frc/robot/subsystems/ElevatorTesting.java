package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.CANBusJNI;
import frc.robot.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorTesting extends SubsystemBase {

    //TODO: SET THESE CONSTANTS
    private static final String CANBUS = "";
    private static final int LEFT_MOTOR_ID = -1;
    private static final int RIGHT_MOTOR_ID = -1;
    private static final double ROTATIONS_TO_HEIGHT_CONVERSION_FACTOR = 0;
    private static final double MIN_HEIGHT = 35; //ASK CAD
    private static final double MAX_HEIGHT = 79; //ASK CAD
    private static final double LEFT_MOTOR_ROTATIONS_AT_MIN_HEIGHT = 0; 
    private static final double RIGHT_MOTOR_ROTATIONS_AT_MIN_HEIGHT = 0;
    private static final double CONTROLLER_DEADBAND = 0.05;
    private static final double INCHES_TOLERANCE = .5;

    //PID + FeedForward Gains
    private static final double LEFT_KP = 0;
    private static final double LEFT_KI = 0;
    private static final double LEFT_KD = 0;
    private static final double LEFT_KS = 0;
    private static final double LEFT_KG = 0;
    private static final double LEFT_KV = 0;

    private static final double RIGHT_KP = 0;
    private static final double RIGHT_KI = 0;
    private static final double RIGHT_KD = 0;
    private static final double RIGHT_KS = 0;
    private static final double RIGHT_KG = 0;
    private static final double RIGHT_KV = 0;
    
    /* Motor Objects */
    private TalonFX leftMotor = new TalonFX(LEFT_MOTOR_ID, CANBUS);
    private TalonFX rightMotor = new TalonFX(RIGHT_MOTOR_ID, CANBUS);

    /* PIDController Objects */
    private PIDController leftController = new PIDController(LEFT_KP, LEFT_KI, LEFT_KD);
    private PIDController rightController = new PIDController(RIGHT_KP, RIGHT_KI, RIGHT_KD);

    /* FeedForward Objects */
    private ElevatorFeedforward leftFF = new ElevatorFeedforward(LEFT_KS, LEFT_KG, LEFT_KV);
    private ElevatorFeedforward rightFF = new ElevatorFeedforward(RIGHT_KS, RIGHT_KG, RIGHT_KV);

    /* Variables to keep track of Encoder Position */
    private double leftEncoderPosition; 
    private double rightEncoderPosition;

    /* Keeps track of what height we set the elevator to be */
    private double heightSetpoint;

    /* Keeps track of the input from the operator controller */
    private double inputVoltage;

    /* Keeps track of the current height of the elevator in inches */
    private double height;

    /* Keeps track of the displacement number of rotations from the min */
    private double rotationsFromMin; 

    /* Booleans to create soft limits */
    private boolean canGoUp;
    private boolean canGoDown;
    private boolean safe;
    private boolean canMoveManual; 
    private boolean PIDsafe;

    /* Boolean to tell whether or not the Elevator is at the desired setpoint */
    private boolean isAtSetPoint;


    /* Constructor for Elevator */
    public ElevatorTesting() {
        /* 
         * Tolerance for the PIDController in the Inches tolerance converted into rotations
         * because the PIDController deals in rotations, but we want to set heights
         * to make it easier on use
         */
        leftController.setTolerance(INCHES_TOLERANCE/ROTATIONS_TO_HEIGHT_CONVERSION_FACTOR);
        leftController.setTolerance(INCHES_TOLERANCE/ROTATIONS_TO_HEIGHT_CONVERSION_FACTOR);

        /* LOGGING */
        Shuffleboard.getTab("Debug").addDouble("Elevator Height", getHeight());
        Shuffleboard.getTab("Debug").addDouble("Height Setpoint", () -> heightSetpoint);

    }

    /* 
     * A lambda function to continuosly get the height of the Elevator
     * for logging purposes
     */
    public DoubleSupplier getHeight() { 
        return () -> height;
    }

    /* A lambda function to continuosly get the rotations from the lowest
     * point
     */
    public DoubleSupplier getRotations(){
        return () -> rotationsFromMin;
    }
    /**
     * @return the amount of rotations the LEFT motor needs to be at
     * for the elevator to be at a height of @param height inches
     */
    public double getRotationsLeftFromHeight(double height) {
        return ((height - MIN_HEIGHT)/ROTATIONS_TO_HEIGHT_CONVERSION_FACTOR) + LEFT_MOTOR_ROTATIONS_AT_MIN_HEIGHT;
    }
    /**
     * @return the amount of rotations the RIGHT motor needs to be at
     * for the elevator to be at a height of @param height inches
     */
    public double getRotationsRightFromHeight(double height) {
        return ((height - MIN_HEIGHT)/ROTATIONS_TO_HEIGHT_CONVERSION_FACTOR) + RIGHT_MOTOR_ROTATIONS_AT_MIN_HEIGHT;
    }
    /**
     * sets the elevator to a height of @param height inches using PID
     */
    public void setHeight(double height){
        //checks safeties -> see periodic method
        if(safe && PIDsafe){
        heightSetpoint = height; //logging purposes
        leftController.setSetpoint(getRotationsLeftFromHeight(height)); //sets setpoint for left motor
        rightController.setSetpoint(getRotationsRightFromHeight(height)); //sets setpoint for right motor
        leftMotor.setVoltage(
            getLeftMotorDefaultFF() + leftController.calculate(leftEncoderPosition)
        ); //sets voltage to be feedforward + whatever PID calculates
        rightMotor.setVoltage(
            getRightMotorDefaultFF() + rightController.calculate(rightEncoderPosition)
        ); //sets voltage to be feedforward + whatever PID calculaes
        } else stop(); //stops motors if not safe!
    }

    /* calculates Feedforward to hold the Elevator in place for the left motor */
    public double getLeftMotorDefaultFF(){
        return leftFF.calculate(0);
    }

    /* calculates Feedforward to hold the Elevator in place for the right motor */
    public double getRightMotorDefaultFF(){
        return rightFF.calculate(0);
    }

    /* method to stop the motors */
    public void stop(){
        leftMotor.set(0);
        rightMotor.set(0);
    }

    /* method to move the elevator using manual control, also includes default feedforward */
    public void manualControl(){
        if(canMoveManual){
        leftMotor.setVoltage(getLeftMotorDefaultFF() + inputVoltage);
        rightMotor.setVoltage(getRightMotorDefaultFF() + inputVoltage);
        } else stop(); //stops motors if not safe
    }


    /*
     * 
     * 
     * COMMANDS
     * 
     */
     

    /* Default command for Elevator is manual control, although we probably won't use this much
     * -Mainly just to set the feedforward
     */
    public Command manualControlCommand(){
        return Commands.run(() -> manualControl());
    }

    /* Command to stop the elevator */
    public Command stopCommand(){
        return Commands.runOnce(() -> stop());
    }

    /* Command to set the height of the elevator */
    public Command setHeightCommand(double height){
        return Commands.run(() -> setHeight(height))
        .until(() -> isAtSetPoint);
    }

    @Override
    public void periodic(){

        /* sets different values for logging and general use */
        leftEncoderPosition = leftMotor.getPosition().getValueAsDouble();
        rightEncoderPosition = rightMotor.getPosition().getValueAsDouble();
        rotationsFromMin = leftEncoderPosition - LEFT_MOTOR_ROTATIONS_AT_MIN_HEIGHT;
        height = rotationsFromMin * ROTATIONS_TO_HEIGHT_CONVERSION_FACTOR + MIN_HEIGHT;

        /* tests if the elevator is where we want it to be */
        isAtSetPoint = leftController.atSetpoint() && rightController.atSetpoint();
    
        /* input from joystick 
         * Multiplied by 12 so it converts to voltage (12v)
         * TODO: make sure this *12 is correct
        */
        inputVoltage = MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), CONTROLLER_DEADBAND) *12;

        /* soft limits */
        canGoUp = height < MAX_HEIGHT;
        canGoDown = height > MIN_HEIGHT;
        safe = canGoUp && canGoDown;
        canMoveManual = (!canGoUp && inputVoltage > 0) || (!canGoDown && inputVoltage < 0);
        PIDsafe = heightSetpoint < MAX_HEIGHT && heightSetpoint > MIN_HEIGHT;

    }
}

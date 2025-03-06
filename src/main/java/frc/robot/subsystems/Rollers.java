package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.GamePiece;

public class Rollers extends SubsystemBase {
    public SparkMax rightMotor;
    public SparkMax leftMotor;
    public final double CORAL_INTAKE_SPEED = 0.5;
    public final double CORAL_OUTTAKE_SPEED = -1;
    public final double ALGAE_INTAKE_SPEED = 0.3;
    public final double ALGAE_OUTTAKE_SPEED = -1;
    public final double ROLLER_STALL_CURRENT = 30; // TODO check/tune
    public final double CORAL_HOLDING_SPEED = 0.1;
    public final double ALGAE_HOLDING_SPEED = 0.5;

    private NetworkTableEntry entry;

    private GamePiece heldPiece = GamePiece.None; // TODO init to Coral for auton? not needed?

    public Rollers() {
        rightMotor = new SparkMax(31, MotorType.kBrushless);
        leftMotor = new SparkMax(30, MotorType.kBrushless);

        var config = new SparkMaxConfig();
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        rightMotor.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        // nt.startServer(); 
        entry = nt.getTable("Testing").getEntry("IsHeld");

    }

    public GamePiece getHeldPiece() {
        return heldPiece;
    }

    public boolean hasCoral() {
        if (getHeldPiece() == GamePiece.Coral){
            return true;
        } else{
            return false;
        }
    }
    public boolean hasAlgae() {
        if (getHeldPiece() == GamePiece.Algae){
            return true;
        } else{
            return false;
        }
    }

    // TODO need to be debounced? probably not?
    public boolean isStalled() {
        return rightMotor.getOutputCurrent() > ROLLER_STALL_CURRENT
                || leftMotor.getOutputCurrent() > ROLLER_STALL_CURRENT;
    }

    public void setSpeed(double speed) {
        setSpeed(speed, speed);
    }

    private void setSpeed(double leftSpeed, double rightSpeed) {
        rightMotor.set(leftSpeed);
        leftMotor.set(rightSpeed);
    }

    private void stopMotors() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stopMotors(), this);
    }

    public Command intakeAlgaeCommand() {
        return Commands.run(() -> setSpeed(ALGAE_INTAKE_SPEED, 0), this)
                .finallyDo(() -> {
                    setSpeed(0);
                    // if(isStalled())
                    this.heldPiece = GamePiece.Algae;
                })
                .until(this::isStalled);
    }

    public Command outtakeAlgaeCommand() {
        return Commands.run(() -> setSpeed(ALGAE_OUTTAKE_SPEED), this)
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.None;
                }).withTimeout(2); // TODO find timeout
    }

    public Command scoreL1() {
        return Commands.run(() -> {
                setSpeed(0, CORAL_OUTTAKE_SPEED);
        }, this)
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.None;
                }).withTimeout(2); // TODO find timeout
    }

    public Command intakeCoralCommand() {
        return Commands.run(() -> setSpeed(CORAL_INTAKE_SPEED), this)
                .finallyDo(() -> {
                    stopMotors();
                    // if(isStalled())
                    this.heldPiece = GamePiece.Coral;
                })
                .until(this::isStalled);
    }

    public Command outtakeCoralCommand() {
        return Commands.run(() -> {
          
                setSpeed(CORAL_OUTTAKE_SPEED);
        }, this)
                .finallyDo(() -> {
                    stopMotors();
                    this.heldPiece = GamePiece.None;
                }).withTimeout(2); // TODO find timeout
    }

    public Command intakeCoralJiggleCommand() {
        return Commands.run(() -> setSpeed(CORAL_INTAKE_SPEED), this)
            .until(this::isStalled)
            .andThen(Commands.sequence(
                Commands.run(() -> setSpeed(-0.1), this)
                .withTimeout(0.1)
                .andThen(intakeCoralCommand())
            )).andThen(Commands.sequence(
                Commands.run(() -> setSpeed(-0.1), this)
                .withTimeout(0.1)
                .andThen(intakeCoralCommand())
            )).finallyDo(() -> {
                this.heldPiece = GamePiece.Coral;
            });
            
    }

    public Command holdCoralCommand() {
        return Commands.run(() -> {
            setSpeed(CORAL_HOLDING_SPEED);
        }, this);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Right Motor Current", rightMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Left Motor Current", leftMotor.getOutputCurrent());
        boolean isHeld = (heldPiece != GamePiece.None)&&!(DriverStation.isAutonomousEnabled());
        
        if(RobotContainer.debugMode && !DriverStation.isFMSAttached()) {
            entry.setBoolean(isHeld);
        }

        Logger.recordOutput("Rollers/Held Piece", heldPiece);
        Logger.recordOutput("Rollers/LeftSpeed", leftMotor.getAppliedOutput());
        Logger.recordOutput("Rollers/RightSpeed", rightMotor.getAppliedOutput());
        Logger.recordOutput("Rollers/RightCurrent", rightMotor.getOutputCurrent());
        Logger.recordOutput("Rollers/LeftCurrent", leftMotor.getOutputCurrent());

    }

    public Command getDefaultCommand() {
        return Commands.run(() -> {
            if(hasCoral()) {
                this.setSpeed(CORAL_HOLDING_SPEED);
            }
    
            if(hasAlgae()) {
                this.setSpeed(ALGAE_HOLDING_SPEED, 0);
            }
        }, this);
    }
}

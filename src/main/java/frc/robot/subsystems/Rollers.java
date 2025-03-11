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
    public SparkMax motor;
    public final double CORAL_INTAKE_SPEED = 0.5;
    public final double CORAL_OUTTAKE_SPEED = -1;
    public final double ROLLER_STALL_CURRENT = 30; 
    public final double CORAL_HOLDING_SPEED = 0.1;
    public final double ALGAE_CLEARING_SPEED = 0.7;

    private NetworkTableEntry entry;

    private GamePiece heldPiece = GamePiece.None; 

    public Rollers() {
        motor = new SparkMax(31, MotorType.kBrushless);

        var config = new SparkMaxConfig();
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        motor.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    public boolean isStalled() {
        return motor.getOutputCurrent() > ROLLER_STALL_CURRENT;
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    private void stopMotor() {
        motor.stopMotor();
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stopMotor(), this);
    }


    public Command clearAlgaeCommand() {
        return Commands.run(() -> setSpeed(ALGAE_CLEARING_SPEED), this)
                .finallyDo(() -> {
                    setSpeed(0);
                })
                .until(this::isStalled);
    }


    public Command scoreL1() {
        return Commands.run(() -> {
                setSpeed(CORAL_OUTTAKE_SPEED);
        }, this)
                .finallyDo(() -> {
                    stopMotor();
                    this.heldPiece = GamePiece.None;
                }).withTimeout(2); // TODO find timeout
    }

    public Command intakeCoralCommand() {
        return Commands.run(() -> setSpeed(CORAL_INTAKE_SPEED), this)
                .finallyDo(() -> {
                    stopMotor();
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
                    stopMotor();
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
        Logger.recordOutput("Rollers/LeftSpeed", motor.getAppliedOutput());
        Logger.recordOutput("Rollers/RightCurrent", motor.getOutputCurrent());

    }

    public Command getDefaultCommand() {
        return Commands.run(() -> {
            if(hasCoral()) {
                this.setSpeed(CORAL_HOLDING_SPEED);
            }
        }, this);
    }
}

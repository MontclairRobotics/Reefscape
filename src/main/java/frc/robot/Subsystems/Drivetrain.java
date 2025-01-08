package frc.robot.Subsystems;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    
    // TalonFX driveFrontLeft = new TalonFX(-1);
    // TalonFX turnFrontLeft = new TalonFX(-1);
    // TalonFX driveFrontRight = new TalonFX(-1);
    // TalonFX turnFrontRight = new TalonFX(-1);
    // TalonFX driveBackLeft = new TalonFX(-1);
    // TalonFX turnBackLeft = new TalonFX(-1);
    // TalonFX driveBackRight = new TalonFX(-1);
    // TalonFX turnBackRight = new TalonFX(-1);
    
    
    public void drive() {

        double xInput = RobotContainer.driverController.getLeftX();
        double yInput = RobotContainer.driverController.getLeftY();
        double rotInput = RobotContainer.driverController.getRightX();

        double xVelocity = Math.pow(xInput, 3) * DriveConstants.MAX_SPEED;
        double yVelocity = Math.pow(yInput, 3) * DriveConstants.MAX_SPEED;
        double rotVelocity = Math.pow(rotInput,3) * DriveConstants.MAX_SPEED;

        final SwerveRequest.FieldCentric fieldRequest = new SwerveRequest.FieldCentric()
            .withDeadband(2) //not sure what these deadbands are for yet
            .withRotationalDeadband(3) 
            .withDriveRequestType(DriveRequestType.Velocity) //Velocity is closed-loop velocity control
            .withSteerRequestType(SteerRequestType.Position); //There's also motionMagicExpo-need to look more into that
        
        final SwerveDrivetrain swerveDrive; //TODO: Configure this

        swerveDrive.setControl(
            fieldRequest.withVelocityX(yVelocity)
            .withVelocityY(xVelocity)
            .withRotationalRate(rotVelocity)
        );
    }

    public Command driveJoystickInput(){
        return Commands.run(() -> drive(), this);
    }
}

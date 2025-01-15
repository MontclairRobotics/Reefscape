package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.ImmutableMomentOfInertia;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.Drivetrain;

public class Constants {
    public class DriveConstants {

        public static final double MAX_SPEED = 4; //TODO: actually set this with units
        public static final double MAX_ROT_SPEED = Math.PI*2;
        public static final double FORWARD_ACCEL = 3; // m / s^2
        public static final double SIDE_ACCEL = 3; //m / s^2
        public static final double ROT_ACCEL = 2; // radians / s^2

    }
    public class DriveConfig {

            //This is probably the PID constants creation thingy for STEER
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(1.91).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        //This is probably the PID constants creation thingy for DRIVE
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Current.ofBaseUnits(1, Amps); //TODO: set correct value

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                
                .withStatorCurrentLimit(60) //TODO: set correct value
                .withStatorCurrentLimitEnable(false) //TODO: set correct value
                .withSupplyCurrentLimit(60)  //TODO: set correct value
                .withSupplyCurrentLimitEnable(false)  //TODO: set correct value
            );
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(60)  //TODO: set correct value
                    .withStatorCurrentLimitEnable(false)  
                    .withSupplyCurrentLimit(60)  //TODO: set correct value
                    .withSupplyCurrentLimitEnable(false)  //TODO: set correct value
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(12); //TODO: set correct value

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.8181818181818183;

        private static final double kDriveGearRatio = 7.363636363636365;
        private static final double kSteerGearRatio = 15.42857142857143;
        private static final Distance kWheelRadius = Distance.ofBaseUnits(1, Inches); //TODO: set correct value

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        public static final int kPigeonId = 1;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = MomentOfInertia.ofBaseUnits(1, KilogramSquareMeters); //TODO: set correct value
        private static final MomentOfInertia kDriveInertia = MomentOfInertia.ofBaseUnits(1, KilogramSquareMeters); //TODO: set correct value
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Voltage.ofBaseUnits(1, Volts); //TODO: set correct value
        private static final Voltage kDriveFrictionVoltage = Voltage.ofBaseUnits(1, Volts); //TODO: set correct value

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio) // how much the drive wheel turns when the angle motor is spun
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent) // How stator current you can apply without slipping
                .withSpeedAt12Volts(kSpeedAt12Volts) //required for open loop control
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);


        // Front Left
        private static final int kFrontLeftDriveMotorId = 3;
        private static final int kFrontLeftSteerMotorId = 2;
        private static final int kFrontLeftEncoderId = 1;
        private static final Angle kFrontLeftEncoderOffset = Radians.of(3); //TODO: actually set value
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(10); //TODO: actually set value
        private static final Distance kFrontLeftYPos = Inches.of(10);//TODO: actually set value

        // Front Right
        private static final int kFrontRightDriveMotorId = 1;
        private static final int kFrontRightSteerMotorId = 0;
        private static final int kFrontRightEncoderId = 0;
        private static final Angle kFrontRightEncoderOffset = Radians.of(3); //TODO: actually set value
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(10); //TODO: set correct value
        private static final Distance kFrontRightYPos = Inches.of(-10); //TODO: set correct value

        // Back Left
        private static final int kBackLeftDriveMotorId = 7;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 3;
        private static final Angle kBackLeftEncoderOffset = Radians.of(3); //TODO: actually set value
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-10); //TODO: actually set value
        private static final Distance kBackLeftYPos = Inches.of(10); //TODO: actually set value

        // Back Right
        private static final int kBackRightDriveMotorId = 5;
        private static final int kBackRightSteerMotorId = 4;
        private static final int kBackRightEncoderId = 2;
        private static final Angle kBackRightEncoderOffset = Radians.of(3);//TODO: actually set value
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-10);//TODO: actually set value
        private static final Distance kBackRightYPos = Inches.of(-10);//TODO: actually set value


        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );

        public static final Matrix<N3, N1> odometryStandardDeviation = VecBuilder.fill(1, 1, 1);
        public static final Matrix<N3, N1> visionStandardDeviation = VecBuilder.fill(1, 1, 1);
        public static final double odometryUpdateFrequency = 100;

        //method to create SwerveDrivetrain object
        public static SwerveDrivetrain<TalonFX, TalonFX, CANcoder> createSwerveDrivetrain(){
            return new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
                TalonFX::new, 
                TalonFX::new, 
                CANcoder::new, 
                DrivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                FrontLeft,
                FrontRight,
                BackLeft,
                BackRight
                );
        }
    }

    
}


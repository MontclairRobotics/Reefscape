package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    
    TalonFX driveFrontLeft = new TalonFX(-1);
    TalonFX turnFrontLeft = new TalonFX(-1);
    TalonFX driveFrontRight = new TalonFX(-1);
    TalonFX turnFrontRight = new TalonFX(-1);
    TalonFX driveBackLeft = new TalonFX(-1);
    TalonFX turnBackLeft = new TalonFX(-1);
    TalonFX driveBackRight = new TalonFX(-1);
    TalonFX turnBackRIght = new TalonFX(-1);
}

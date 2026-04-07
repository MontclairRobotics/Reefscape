package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain {
    SwerveDriveKinematics swerveDriveKinematics;
    
    SwerveDriveOdometry swerveOdometry;
    Gyroscope gyro; //Temporary object / class
    SwerveModule[] swervemodules; //Temporary object / class

    public Drivetrain(SwerveDriveKinematics kinematics, Gyroscope gyro, ) {
        swerveDriveKinematics = kinematics;
    } 

    public void drive(double xVel, double yVel, double rotVel) {
        ChassisSpeeds speeds = new ChassisSpeeds(xVel, yVel,rotVel);  
        SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(speeds);
    }


}

public class SwerveDrive extends SubsystemBase {
    
}
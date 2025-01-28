package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.util.DriveConfig;

import java.util.Arrays;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private SparkMax rightMotor;
    private SparkMax leftMotor;

    public Rollers() {
    rightMotor = new SparkMax(0, null);
    leftMotor = new SparkMax(0, null);
}

    public void setIntakeSpeed () {
    rightMotor.set (1);
    leftMotor.set (1);
    }   
}

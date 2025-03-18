package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PointWheelsAtCommand extends Command {
  private Rotation2d direction;
  private double startTime = 0;

  public PointWheelsAtCommand(Rotation2d direction) {
    this.direction = direction;
    addRequirements(RobotContainer.drivetrain); //requires the drivetrain
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    // Nothing to do
  }

  @Override
  public void execute() {
    RobotContainer.drivetrain.alignWheels(direction);
  }

  @Override
  public boolean isFinished() {
    SwerveModuleState[] states = RobotContainer.drivetrain.getState().ModuleStates;
    for (var i = 0; i < states.length; i++) {
        SwerveModuleState state = states[i];
        double diff = Math.abs(state.angle.getDegrees() - direction.getDegrees()) % 180.0;
        Logger.recordOutput("PointWheelsAt/state" + i, state.angle.getDegrees());
        Logger.recordOutput("PointWheelsAt/direction" + i, direction.getDegrees());
        Logger.recordOutput("PointWheelsAt/diff" + i, diff);
        if (diff > 1 && diff < 179) {
            return false;
        }
    }
    System.out.println("********************* PointWheelsAtCommand finished in " + (Timer.getFPGATimestamp() - startTime) + " seconds");
    return true;
  }

}

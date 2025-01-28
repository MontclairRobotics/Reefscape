package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
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

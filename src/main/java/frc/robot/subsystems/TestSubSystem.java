package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubSystem extends SubsystemBase {
    private TalonFX talonFX;

    /**
     * Constructor
     */
    public TestSubSystem() {
        // Create motors
        talonFX = new TalonFX(10, "");
        // TalonFXConfigurator configurator = talonFX.getConfigurator();
        // TalonFXConfiguration configuration = new TalonFXConfiguration();
        // configuration.CurrentLimits.
    }




    /**
     * All periodic does is set voltage to 12
     */
    @Override
    public void periodic() {
        talonFX.setVoltage(2);
        System.out.println("motorVoltage: " + talonFX.getMotorVoltage());
    }

    /**
     * Simulation just gets voltage and logs it
     */
    @Override
    public void simulationPeriodic() {
        var talonFXSim = talonFX.getSimState();

        talonFXSim.setSupplyVoltage(13.5);

        var simMotorVoltage = talonFXSim.getMotorVoltage();
        System.out.println("simMotorVoltage: " + simMotorVoltage);
   }
}

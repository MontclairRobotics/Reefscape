package frc.robot.subsystems;

import edu.wpi.first.hal.util.AllocationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class Ratchet extends SubsystemBase {
    private final Servo leftServo;
    private final Servo rightServo;
    public final double DISENGAGED_POSITION = 0;
    public final double ENGAGED_POSITION = 0.25;
    public boolean ratchetEngaged = false;
    public Ratchet() {
        leftServo = new Servo(3);
        rightServo = new Servo(10);
    }

    private void setServoPosition(Servo servo, double position) {
        try {
            if (position < 0 || position > 1) {
                throw new IllegalArgumentException("Servo position out of bounds: " + position);
            }
            servo.set(position);
        } catch (IllegalArgumentException e) {
            DriverStation.reportError("Invalid servo position: " + position + " for servo " + servo, true);
        }  catch (AllocationException e) {
            DriverStation.reportError("Invalid servo position: " + position + " for servo " + servo, true);
        }catch (Exception e) {
            DriverStation.reportError("Error setting servo position: " + e.getMessage(), true);
        }
    }

    public Command engageServos() {
        return Commands.run(() -> {
            try {
                setServoPosition(leftServo, ENGAGED_POSITION);
                setServoPosition(rightServo, -ENGAGED_POSITION);
                this.ratchetEngaged = true;
            } catch (Exception e) {
                DriverStation.reportError("Failed to engage servos: " + e.getMessage(), true);
            }
        }, this);
    }

    public Command disengageServos() {
        return Commands.run(() -> {
            try {
                setServoPosition(leftServo, DISENGAGED_POSITION);
                setServoPosition(rightServo, -DISENGAGED_POSITION);
                this.ratchetEngaged = false;
            } catch (Exception e) {
                DriverStation.reportError("Failed to disengage servos: " + e.getMessage(), true);
            }
        }, this);

    }
}

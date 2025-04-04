package frc.robot.subsystems;

import edu.wpi.first.hal.util.AllocationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.leds.LEDs;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.util.Color;

public class Ratchet extends SubsystemBase {
    public  Servo leftServo;
    public  Servo rightServo;
    public final double LEFT_DISENGAGED_POSITION = 0;
    public final double LEFT_ENGAGED_POSITION = 0.2;
    public final double LEFT_AFTER_POSITION = 0.2;
    public final double RIGHT_DISENGAGED_POSITION = 0.3;
    public final double RIGHT_ENGAGED_POSITION = 0; //this is correct
    public final double RIGHT_AFTER_POSITION = 0; //this is correct
    public boolean ratchetEngaged = false;
    public Ratchet() {
        leftServo = new Servo(8);
        rightServo = new Servo(6);
    }

    private void setServoPosition(Servo servo, double position) {
        // try {
        //     if (position < 0 || position > 1) {
        //         throw new IllegalArgumentException("Servo position out of bounds: " + position);
        //     }
        //     servo.set(position);
        // } catch (IllegalArgumentException e) {
        //     DriverStation.reportError("Invalid servo position: " + position + " for servo " + servo, true);
        // }  catch (AllocationException e) {
        //     DriverStation.reportError("Invalid servo position: " + position + " for servo " + servo, true);
        // }catch (Exception e) {
        //     DriverStation.reportError("Error setting servo position: " + e.getMessage(), true);
        // }
        servo.set(position);
    }

    public Command engageServos() {
        return Commands.runOnce(() -> {
            try {
                System.out.println("engaging servos");
                setServoPosition(leftServo, LEFT_ENGAGED_POSITION);
                setServoPosition(rightServo,  RIGHT_ENGAGED_POSITION);
                this.ratchetEngaged = true;
            } catch (Exception e) {
                DriverStation.reportError("Failed to engage servos: " + e.getMessage(), true);
            }
        }, this)
        .alongWith(RobotContainer.leds.playLEDPatternCommand(LEDs.blink(Color.kWhite), 0.5));
    }

    public Command afterEngageServos() {
        return Commands.runOnce(() -> {
            try {
                System.out.println("running after servos");
                setServoPosition(leftServo, LEFT_AFTER_POSITION);
                setServoPosition(rightServo,  RIGHT_AFTER_POSITION);
                this.ratchetEngaged = true;
            } catch (Exception e) {
                DriverStation.reportError("Failed to after engage servos: " + e.getMessage(), true);
            }
        }, this);
    }

    public Command disengageServos() {
        return Commands.runOnce(() -> {
            try {
                System.out.println("disengage servos");
                setServoPosition(leftServo, LEFT_DISENGAGED_POSITION);
                setServoPosition(rightServo, RIGHT_DISENGAGED_POSITION);
                this.ratchetEngaged = false;
            } catch (Exception e) {
                DriverStation.reportError("Failed to disengage servos: " + e.getMessage(), true);
            }
        }, this);

    }
}

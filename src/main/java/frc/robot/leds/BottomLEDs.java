package frc.robot.leds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class BottomLEDs extends LEDs{

    public BottomLEDs() {
        led = new AddressableLED(PORT);
        led.setLength(LENGTH);
        ledBuffer = new AddressableLEDBuffer(LENGTH);
        led.start();

    public BottomLEDs() {
        super();

    }

    // TODO this crashes a simulator. Is it needed?
    // AddressableLEDBufferView m_left = ledBuffer.createView(0, 59);

    // The section of the strip on the right side of the robot.
    // This section spans LEDs from index 60 through index 119, inclusive.
    // This view is reversed to cancel out the serpentine arrangement of the
    // physical LED strip on the robot.
    // AddressableLEDBufferView m_right = ledBuffer.createView(60, 119).reversed();

    public void rainbowProgress() {
        final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
        final LEDPattern progress = LEDPattern
                .progressMaskLayer(() -> RobotContainer.elevator.getHeight() / Elevator.ELEVATOR_MAX_HEIGHT);
        LEDPattern pattern = m_rainbow.mask(progress);

        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);

        // for later, trying to make the rainbow scroll ->
        // Map<Number, Color> maskSteps = Map.of(0, Color.kWhite, 0.5, Color.kBlack);
        // LEDPattern mask =
        // LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(0.25));

    }

    public void progressBar() {
        // LEDPattern progess = LEDPattern.progressMaskLayer(() -> Elevator.getHeight()
        // / Elevator.getMaxHeight());;
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    // TODO maybe make this an enum or something, not sure tbh
    public Command playPatternCommand(LEDPattern pattern) {
        return Commands.run(() -> pattern.applyTo(ledBuffer), this).ignoringDisable(true);
    }
}
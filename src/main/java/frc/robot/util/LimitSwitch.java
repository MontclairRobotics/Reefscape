package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public class LimitSwitch implements Sendable {
  /** x ^ i = r ----------.----- F F | F T F | T F T | T T T | F */
  private final boolean invert;

  public DigitalInput dio;
  private boolean value;

  public LimitSwitch(int channel, boolean invert) {
    this.invert = invert;
    if (RobotBase.isReal()) {
      dio = new DigitalInput(channel);
    }
  }

  public boolean get() {
    if (RobotBase.isReal())
      return dio.get() ^ invert;
    else
      return value;
  }

  public int getChannel() {
    return dio.getChannel();
  }

  /**
   * Set the simulated value of this limit switch. Reports an error if called with
   * a non-simulation
   * robot.
   */
  public void set(boolean value) {
    if (RobotBase.isReal()) {
      System.out.println(
          "Cannot set the value of a digital limit switch when not in simulation mode!");
      return;
    }

    this.value = value;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    dio.initSendable(builder);
  }
}

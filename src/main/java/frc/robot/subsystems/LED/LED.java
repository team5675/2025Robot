package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static final int port1 = 0;
  private static final int kLength = 90;

  private final AddressableLED led1;

  private final AddressableLEDBuffer ledBuffer;
  public LEDAnimation currentAnimation = null;

  private double globalBrightness = 1;

  private LED() {
    led1 = new AddressableLED(port1);
    led1.setLength(kLength);
    led1.start();

    ledBuffer = new AddressableLEDBuffer(kLength);
  }

  /**
   * Sets the current animation for both LED strips.
   * This will stop any currently running animation.
   * 
   * @param animation The animation to run
   */
  public void setAnimation(LEDAnimation animation) {
    // if (!RobotContainer.getLEDsEnabled()) {
    //   return;
    // }

    if (currentAnimation != null) {
      currentAnimation.end();
    }

    currentAnimation = animation;
    if (currentAnimation != null) {
      currentAnimation.init(this);
    }
  }

  public AddressableLEDBuffer getBuffer() {
    return ledBuffer;
  }

  public int getLength() {
    return kLength;
  }

  @Override
  public void periodic() {
    // if (!RobotContainer.getLEDsEnabled()) {
    //   return;
    // }

    if (currentAnimation != null) {
      currentAnimation.execute();
    }

    applyGlobalBrightness();

    led1.setData(ledBuffer);
  }

  public void stopPattern() {
    setAnimation(null);
  }

  // Global brightness
  public void setGlobalBrightness(double brightness) {
    this.globalBrightness = Math.min(1.0, Math.max(0.0, brightness));
  }

  public double getGlobalBrightness() {
    return globalBrightness;
  }

  private void applyGlobalBrightness() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      int r = (int) (ledBuffer.getRed(i) * globalBrightness);
      int g = (int) (ledBuffer.getGreen(i) * globalBrightness);
      int b = (int) (ledBuffer.getBlue(i) * globalBrightness);

      ledBuffer.setRGB(i, r, g, b);
    }
  }

  private static LED instance;

  public static LED getInstance() {
    if (instance == null) {
      instance = new LED();
    }
    return instance;
  }
}
package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class Blink implements LEDAnimation {
    private final RGB color;
    private final double offTime;
    private final double onTime;
    private double lastUpdateTime;
    private boolean isOn;
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;

    /**
     * Constructs a blinking LED animation with a custom RGB color.
     *
     * @param color The RGB color for the blink effect.
     * @param offTime How long the LEDs stay off before turning on.
     * @param onTime How long the LEDs stay on before turning off.
     */
    public Blink(RGB color, double offTime, double onTime) {
        this.color = color;
        this.offTime = offTime;
        this.onTime = onTime;
        this.isOn = false;
    }

    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
        this.lastUpdateTime = Timer.getFPGATimestamp();
        this.isOn = false;
        
        // Initialize LEDs to OFF state
        turnOff();
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double elapsedTime = now - lastUpdateTime;

        if (isOn && elapsedTime >= onTime) {
            turnOff();
            lastUpdateTime = now;
        } else if (!isOn && elapsedTime >= offTime) {
            turnOn();
            lastUpdateTime = now;
        }
    }

    @Override
    public void end() {
        // Turn off all LEDs when animation ends
        turnOff();
    }

    private void turnOn() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, color.r, color.g, color.b);
        }
        isOn = true;
    }

    private void turnOff() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        isOn = false;
    }
}
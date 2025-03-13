package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class SolidColor implements LEDAnimation {
    private final RGB color;
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;

    /**
     * Constructs a solid color animation.
     *
     * @param color The color to display
     */
    public SolidColor(RGB color) {
        this.color = color;
    }

    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
    }

    @Override
    public void execute() {
        // Apply the color to every LED
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, color.r, color.g, color.b);
        }
    }

    @Override
    public void end() {
        // Nothing specific to clean up
    }
}
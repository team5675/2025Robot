package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class Alert implements LEDAnimation {
    private final RGB color1;
    private final RGB color2;
    private final double blinkRate; // Blinks per second
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private double lastUpdateTime = 0.0;
    private boolean isColor1 = true;
    private double timeInState = 0.0;

    /**
     * Constructs an alternating color alert animation.
     *
     * @param color1     The first color
     * @param color2     The second color
     * @param blinkRate  Number of complete blink cycles per second
     */
    public Alert(RGB color1, RGB color2, double blinkRate) {
        this.color1 = color1;
        this.color2 = color2;
        this.blinkRate = blinkRate;
    }

    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
        this.lastUpdateTime = Timer.getFPGATimestamp();
        this.isColor1 = true;
        this.timeInState = 0.0;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastUpdateTime;
        lastUpdateTime = now;
        
        // Track time in current state
        timeInState += dt;
        
        // Check if we should switch colors
        // Each color is displayed for 1/(2*blinkRate) seconds
        if (timeInState >= 1.0 / (2 * blinkRate)) {
            isColor1 = !isColor1;
            timeInState = 0.0;
        }
        
        // Determine which color to display
        RGB activeColor = isColor1 ? color1 : color2;
        
        // Apply the active color to every LED
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, activeColor.r, activeColor.g, activeColor.b);
        }
    }

    @Override
    public void end() {}
}
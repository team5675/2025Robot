package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class RainbowPulse implements LEDAnimation {
    // Rainbow type enum (imported from RainbowShootingLines)
    public enum RainbowType {
        RGB_RAINBOW,  // Standard bright rainbow colors
        PASTEL_RAINBOW // Softer pastel rainbow colors
    }
    
    private final RainbowType rainbowType;
    private final double minBrightness;
    private final double maxBrightness;
    private final double pulseCycleTime;
    private final double rainbowCycleTime;
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private double pulsePhase = 0.0;
    private double rainbowPhase = 0.0;
    private double lastUpdateTime = 0.0;
    private double lastPulseResetTime = 0.0;
    private double lastPulsePhase = 0.0;

    /**
     * Constructs a pulsing LED animation with cycling rainbow colors.
     *
     * @param rainbowType     The type of rainbow colors to use (RGB_RAINBOW or PASTEL_RAINBOW)
     * @param minBrightness   The minimum brightness (0.0 to 1.0) during the pulse cycle.
     * @param maxBrightness   The maximum brightness (0.0 to 1.0) during the pulse cycle.
     * @param pulseCycleTime  The total time in seconds for one full pulse cycle.
     * @param rainbowCycleTime The total time in seconds for one full rainbow color cycle.
     *                        Set to 0 to change color on every brightness pulse.
     */
    public RainbowPulse(RainbowType rainbowType, double minBrightness, double maxBrightness, 
                        double pulseCycleTime, double rainbowCycleTime) {
        this.rainbowType = rainbowType;
        this.minBrightness = minBrightness;
        this.maxBrightness = maxBrightness;
        this.pulseCycleTime = pulseCycleTime;
        this.rainbowCycleTime = rainbowCycleTime; // Can be 0 now
    }

    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
        this.lastUpdateTime = Timer.getFPGATimestamp();
        this.lastPulseResetTime = this.lastUpdateTime;
        this.pulsePhase = 0.0;
        this.rainbowPhase = 0.0;
        this.lastPulsePhase = 0.0;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastUpdateTime;
        lastUpdateTime = now;

        // Calculate the previous pulse phase
        double previousPulsePhase = pulsePhase;

        // Increase pulse phase such that a full cycle (2π) happens in pulseCycleTime seconds.
        pulsePhase += (2 * Math.PI * dt / pulseCycleTime);
        pulsePhase %= (2 * Math.PI); // Keep phase within [0, 2π)

        // Check if we've completed a pulse cycle (crossed from high to low)
        boolean pulseCompleted = (previousPulsePhase > Math.PI / 2 && pulsePhase <= Math.PI / 2);

        // Update rainbow phase based on mode
        if (rainbowCycleTime <= 0) {
            // Change color on every brightness pulse
            if (pulseCompleted) {
                // Increment color by a fixed amount when pulse completes
                rainbowPhase += 0.15; // Adjust this value to change color step size
                rainbowPhase %= 1.0;
            }
        } else {
            // Continuous rainbow cycling over time
            rainbowPhase += (dt / rainbowCycleTime);
            rainbowPhase %= 1.0; // Keep phase within [0, 1.0)
        }

        // Sine oscillates from -1 to 1; normalize it to [0, 1].
        double brightnessFactor = (Math.sin(pulsePhase) + 1) / 2.0;
        // Map the normalized value to the specified brightness range.
        double currentBrightness = minBrightness + (maxBrightness - minBrightness) * brightnessFactor;

        // Get the current rainbow color
        RGB color = getRainbowColor(rainbowPhase);

        // Apply the same color to all LEDs with the current brightness
        for (int i = 0; i < buffer.getLength(); i++) {
            int adjustedRed = (int) (color.r * currentBrightness);
            int adjustedGreen = (int) (color.g * currentBrightness);
            int adjustedBlue = (int) (color.b * currentBrightness);
            
            buffer.setRGB(i, adjustedRed, adjustedGreen, adjustedBlue);
        }
    }

    @Override
    public void end() {
        // Nothing to clean up for this animation
    }
    
    // Get a rainbow color based on hue (0.0 to 1.0)
    private RGB getRainbowColor(double hue) {
        // Keep hue between 0.0 and 1.0
        hue = hue % 1.0;
        if (hue < 0) hue += 1.0;
        
        // Convert to HSV color space, then to RGB
        double h = hue * 6.0;
        int i = (int) Math.floor(h);
        double f = h - i;
        double q = 1 - f;
        
        int r, g, b;
        
        switch (i % 6) {
            case 0: r = 255; g = (int) (255 * f); b = 0; break;
            case 1: r = (int) (255 * q); g = 255; b = 0; break;
            case 2: r = 0; g = 255; b = (int) (255 * f); break;
            case 3: r = 0; g = (int) (255 * q); b = 255; break;
            case 4: r = (int) (255 * f); g = 0; b = 255; break;
            case 5: r = 255; g = 0; b = (int) (255 * q); break;
            default: r = 0; g = 0; b = 0; break;
        }
        
        // For pastel rainbow, blend with white
        if (rainbowType == RainbowType.PASTEL_RAINBOW) {
            // Pastels are created by mixing with white and reducing saturation
            // Blend with white (add white)
            r = r + (255 - r) / 2;
            g = g + (255 - g) / 2;
            b = b + (255 - b) / 2;
        }
        
        return new RGB(r, g, b);
    }
}
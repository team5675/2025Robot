package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * A customizable rainbow animation for addressable LED strips.
 * Features adjustable brightness, speed, and rainbow type options.
 */
public class CustomizableRainbow implements LEDAnimation {
    // Rainbow type enum
    public enum RainbowType {
        STANDARD,    // Full saturation, full brightness rainbow
        PASTEL,      // Softer, more pastel colors (mixed with white)
        NEON,        // Vivid, high-contrast colors
        COOL,        // Blues and greens
        COOL_PURPLE, 
        WARM         // Reds, oranges, and yellows
    }
    
    // Pattern type enum
    public enum PatternType {
        CONTINUOUS,  // Seamless rainbow that flows across the entire strip
        SEGMENTED,   // Strip divided into rainbow segments
        MIRRORED     // Rainbow pattern mirrored at the center of the strip
    }
    
    // Animation direction
    public enum Direction {
        FORWARD,     // Rainbow moves from start to end of strip
        BACKWARD,    // Rainbow moves from end to start of strip
        STATIC       // Rainbow doesn't move (fixed in place)
    }
    
    private final RainbowType rainbowType;
    private final PatternType patternType;
    private final Direction direction;
    
    private double animationSpeed;    // Cycles per second (1.0 = one full cycle per second)
    private double localBrightness;   // 0.0 to 1.0, separate from system global brightness
    private int segmentSize;          // For SEGMENTED pattern type
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private double startTime;
    
    /**
     * Creates a customizable rainbow animation.
     * 
     * @param rainbowType     The color scheme to use
     * @param patternType     The pattern of the rainbow across the strip
     * @param direction       The direction of animation flow
     * @param speed           Speed in cycles per second (1.0 = full cycle per second)
     * @param brightness      Local brightness from 0.0 (off) to 1.0 (full brightness)
     * @param segmentSize     Size of segments for SEGMENTED pattern type (ignored for other types)
     */
    public CustomizableRainbow(RainbowType rainbowType, PatternType patternType, Direction direction,
                              double speed, double brightness, int segmentSize) {
        this.rainbowType = rainbowType;
        this.patternType = patternType;
        this.direction = direction;
        this.animationSpeed = Math.max(0.01, speed);  // Minimum speed to prevent freezing
        this.localBrightness = Math.min(1.0, Math.max(0.0, brightness));  // Clamp to valid range
        this.segmentSize = Math.max(1, segmentSize);  // Minimum size of 1
    }
    
    /**
     * Creates a customizable rainbow animation with default segment size.
     */
    public CustomizableRainbow(RainbowType rainbowType, PatternType patternType, Direction direction,
                              double speed, double brightness) {
        this(rainbowType, patternType, direction, speed, brightness, 10);
    }
    
    /**
     * Creates a standard continuous forward rainbow with default settings.
     */
    public CustomizableRainbow() {
        this(RainbowType.STANDARD, PatternType.CONTINUOUS, Direction.FORWARD, 0.5, 1.0);
    }
    
    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
        this.startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - startTime;
        
        // Current phase determines the offset in the rainbow cycle (0.0 to 1.0)
        double phase = (elapsedTime * animationSpeed) % 1.0;
        if (direction == Direction.BACKWARD) {
            phase = 1.0 - phase;
        } else if (direction == Direction.STATIC) {
            phase = 0.0;
        }
        
        // Apply the pattern to each LED
        int length = buffer.getLength();
        for (int i = 0; i < length; i++) {
            double hue = calculateHue(i, length, phase);
            RGB color = getRainbowColor(hue);
            
            // Apply local brightness
            int r = (int)(color.r * localBrightness);
            int g = (int)(color.g * localBrightness);
            int b = (int)(color.b * localBrightness);
            
            buffer.setRGB(i, r, g, b);
        }
    }
    
    /**
     * Calculate the hue for a specific LED based on its position and the pattern type.
     */
    private double calculateHue(int ledIndex, int stripLength, double phase) {
        switch (patternType) {
            case CONTINUOUS:
                // Hue varies continuously across the strip
                return (ledIndex / (double)stripLength + phase) % 1.0;
                
            case SEGMENTED:
                // Strip is divided into segments, each segment gets its own rainbow
                int segmentIndex = ledIndex / segmentSize;
                double positionWithinSegment = (ledIndex % segmentSize) / (double)segmentSize;
                return (positionWithinSegment + phase + (segmentIndex * 0.25)) % 1.0;
                
            case MIRRORED:
                // Rainbow is mirrored at the center
                double normalizedPos;
                if (ledIndex < stripLength / 2) {
                    normalizedPos = ledIndex / (double)(stripLength / 2);
                } else {
                    normalizedPos = 2.0 - (ledIndex / (double)(stripLength / 2));
                }
                return (normalizedPos + phase) % 1.0;
                
            default:
                return (ledIndex / (double)stripLength + phase) % 1.0;
        }
    }
    
    /**
     * Get an RGB color for a specific hue value (0.0 to 1.0) based on the selected rainbow type.
     */
    private RGB getRainbowColor(double hue) {
        // Ensure hue is in 0.0-1.0 range
        hue = hue % 1.0;
        if (hue < 0) hue += 1.0;
        
        // For specific rainbow types, adjust the hue range or saturation
        switch (rainbowType) {
            case COOL:
                // Limit to blues and greens (0.3 - 0.7 in hue space)
                hue = 0.3 + (hue * 0.3);
                break;

            case COOL_PURPLE:
                // Limit to blues and greens (0.3 - 0.7 in hue space)
                hue = 0.3 + (hue * 0.5);
                break;
                
            case WARM:
                // Limit to reds, oranges, yellows (0.9 - 0.2 in hue space)
                if (hue < 0.5) {
                    hue = 0.9 + (hue * 0.2); // 0.9 - 1.0 (reds)
                } else {
                    hue = (hue - 0.5) * 0.4; // 0.0 - 0.2 (yellows to oranges)
                }
                break;
                
            default:
                // Keep full hue range for other types
                break;
        }
        
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
        
        // Apply type-specific color adjustments
        switch (rainbowType) {
            case PASTEL:
                // Pastel colors are created by blending with white
                r = r + (255 - r) / 2;
                g = g + (255 - g) / 2;
                b = b + (255 - b) / 2;
                break;
                
            case NEON:
                // Neon colors have increased contrast and brightness
                r = Math.min(255, (int)(r * 1.2));
                g = Math.min(255, (int)(g * 1.2));
                b = Math.min(255, (int)(b * 1.2));
                break;
                
            default:
                // No adjustment for standard
                break;
        }
        
        return new RGB(r, g, b);
    }
    
    @Override
    public void end() {
        // Clear the LED buffer if needed
        if (buffer != null) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
    }
    
    // Getters and setters for runtime adjustments
    
    public void setSpeed(double speed) {
        this.animationSpeed = Math.max(0.01, speed);
    }
    
    public double getSpeed() {
        return animationSpeed;
    }
    
    public void setBrightness(double brightness) {
        this.localBrightness = Math.min(1.0, Math.max(0.0, brightness));
    }
    
    public double getBrightness() {
        return localBrightness;
    }
    
    public void setSegmentSize(int size) {
        this.segmentSize = Math.max(1, size);
    }
    
    public int getSegmentSize() {
        return segmentSize;
    }
}
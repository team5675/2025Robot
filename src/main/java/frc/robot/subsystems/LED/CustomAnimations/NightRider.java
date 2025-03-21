package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.Random;

public class NightRider implements LEDAnimation {
    // Scan style enum - determines how the scanning light moves
    public enum ScanStyle {
        LINEAR,     // Moves from one end to the other
        BOUNCE,     // Bounces back and forth between ends
        PING_PONG,  // Like bounce but smoother transitions
        DUAL_SCAN,  // Two scanners moving in opposite directions
        SPLIT_SCAN  // Starts in middle and splits outward, then returns
    }
    
    // Tail style enum - determines how the light fades
    public enum TailStyle {
        FADE_OUT,       // Gradually fades out behind the scanner
        SHARP_CUT,      // Sharp edge with no fade
        PULSE,          // Pulsing brightness behind the scanner
        GLOW,           // Long glow effect
        EXPONENTIAL     // Exponential fade (faster at first, then slower)
    }
    
    // Direction behavior enum - determines scanning behavior
    public enum DirectionBehavior {
        FORWARD,        // Always moves in positive direction
        BACKWARD,       // Always moves in negative direction
        ALTERNATE,      // Alternates between forward and backward
        RANDOM          // Random direction for each cycle
    }
    
    // Rainbow type enum - determines color behavior
    public enum RainbowType {
        NONE,           // Uses the specified color
        RGB_RAINBOW,    // Standard bright rainbow colors
        PASTEL_RAINBOW, // Softer pastel rainbow colors
        FIRE,           // Fire-like colors (red, orange, yellow)
        OCEAN           // Ocean-like colors (blue, teal, green)
    }
    
    private final ScanStyle scanStyle;
    private final TailStyle tailStyle;
    private final DirectionBehavior directionBehavior;
    private final RainbowType rainbowType;
    private final Color color;
    private final double speed; // Speed factor (0.0 to 10.0)
    
    private final int eyeSize; // Size of the eye/scanner
    private final int tailLength; // Length of the tail
    private final double cycleTime; // Time for one complete cycle in seconds
    private final double tailBrightness; // Brightness of the tail (0.0 to 1.0)
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private double startTime;
    private double lastUpdateTime;
    private double position = 0;
    private double direction = 1;
    private Random random = new Random();
    private double[] brightness; // Array to store brightness values
    private double rainbowPhase = 0;
    
    /**
     * Creates a Night Rider scanning animation
     * 
     * @param scanStyle The style of scanning motion
     * @param tailStyle The style of tail/fade effect
     * @param directionBehavior How the scanner direction changes
     * @param rainbowType The type of rainbow effect, or NONE to use color
     * @param color The color to use (ignored if rainbowType is not NONE)
     * @param speed Speed factor (0.0 to 10.0)
     * @param eyeSize Size of the eye/scanner (pixels)
     * @param tailLength Length of the tail (pixels)
     * @param cycleTime Time for one complete cycle (seconds)
     * @param tailBrightness Brightness of the tail (0.0 to 1.0)
     */
    public NightRider(
            ScanStyle scanStyle,
            TailStyle tailStyle,
            DirectionBehavior directionBehavior,
            RainbowType rainbowType,
            Color color,
            double speed,
            int eyeSize,
            int tailLength,
            double cycleTime,
            double tailBrightness) {
        this.scanStyle = scanStyle;
        this.tailStyle = tailStyle;
        this.directionBehavior = directionBehavior;
        this.rainbowType = rainbowType;
        this.color = color;
        this.speed = Math.max(0.1, Math.min(10.0, speed)); // Clamp speed between 0.1 and 10.0
        this.eyeSize = Math.max(1, eyeSize);
        this.tailLength = Math.max(0, tailLength);
        this.cycleTime = Math.max(0.1, cycleTime);
        this.tailBrightness = Math.max(0.0, Math.min(1.0, tailBrightness));
    }
    
    /**
     * Simplified constructor with default values
     */
    public NightRider(
            ScanStyle scanStyle,
            TailStyle tailStyle,
            DirectionBehavior directionBehavior,
            RainbowType rainbowType,
            Color color,
            double speed) {
        this(scanStyle, tailStyle, directionBehavior, rainbowType, color, speed, 3, 10, 2.0, 0.5);
    }
    
    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
        this.brightness = new double[buffer.getLength()];
        this.startTime = Timer.getFPGATimestamp();
        this.lastUpdateTime = startTime;
        this.position = 0;
        
        // Initialize direction based on behavior
        switch (directionBehavior) {
            case FORWARD:
                this.direction = 1;
                break;
            case BACKWARD:
                this.direction = -1;
                break;
            case ALTERNATE:
            case RANDOM:
                this.direction = random.nextBoolean() ? 1 : -1;
                break;
        }
        
        // Initialize brightness array
        Arrays.fill(brightness, 0.0);
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        
        // Update rainbow phase
        if (rainbowType != RainbowType.NONE) {
            rainbowPhase = (rainbowPhase + dt * speed * 0.5) % 1.0;
        }
        
        int length = buffer.getLength();
        
        // Update position based on scan style
        switch (scanStyle) {
            case LINEAR:
                updateLinearScan(dt, length);
                break;
            case BOUNCE:
                updateBounceScan(dt, length);
                break;
            case PING_PONG:
                updatePingPongScan(dt, length);
                break;
            case DUAL_SCAN:
                updateDualScan(dt, length);
                break;
            case SPLIT_SCAN:
                updateSplitScan(dt, length);
                break;
        }
        
        // Apply the calculated brightness to the buffer
        applyBrightnessToBuffer();
    }
    
    private void updateLinearScan(double dt, int length) {
        // Linear scan: moves from one end to the other, then wraps
        double moveAmount = dt * ((length + eyeSize + tailLength) / cycleTime) * speed;
        position = (position + moveAmount * direction) % (length + eyeSize + tailLength);
        if (position < 0) position += (length + eyeSize + tailLength);
        
        // Reset brightness array
        Arrays.fill(brightness, 0.0);
        
        // Apply eye and tail brightness
        applyEyeAndTail(position, direction);
        
        // Update direction based on behavior
        if (directionBehavior == DirectionBehavior.ALTERNATE) {
            if (position >= (length + eyeSize + tailLength) - 1 || position <= 0) {
                direction *= -1;
            }
        } else if (directionBehavior == DirectionBehavior.RANDOM) {
            if (position >= (length + eyeSize + tailLength) - 1 || position <= 0) {
                direction = random.nextBoolean() ? 1 : -1;
            }
        }
    }
    
    private void updateBounceScan(double dt, int length) {
        // Bounce scan: bounces back and forth between ends
        double moveAmount = dt * (length / cycleTime) * speed;
        position += moveAmount * direction;
        
        // Check for bounce
        if (position >= length - eyeSize || position <= 0) {
            direction *= -1;
            position = Math.max(0, Math.min(length - eyeSize, position));
            
            // Update direction based on behavior at bounce points
            if (directionBehavior == DirectionBehavior.RANDOM) {
                direction = random.nextBoolean() ? 1 : -1;
            } else if (directionBehavior == DirectionBehavior.FORWARD) {
                direction = 1;
            } else if (directionBehavior == DirectionBehavior.BACKWARD) {
                direction = -1;
            }
        }
        
        // Reset brightness array
        Arrays.fill(brightness, 0.0);
        
        // Apply eye and tail brightness
        applyEyeAndTail(position, direction);
    }
    
    private void updatePingPongScan(double dt, int length) {
        // Ping pong scan: like bounce but with smoother transitions
        double totalDistance = length * 2 - eyeSize * 2;
        double moveAmount = dt * (totalDistance / cycleTime) * speed;
        
        // Keep track of position in the cycle (0 to 1)
        double cycle = (Timer.getFPGATimestamp() - startTime) % cycleTime / cycleTime;
        
        // Calculate position using sine wave for smooth transition
        double normalizedPos = (Math.sin(cycle * 2 * Math.PI) + 1) / 2;
        position = normalizedPos * (length - eyeSize);
        
        // Direction is the derivative of position
        direction = Math.cos(cycle * 2 * Math.PI) >= 0 ? 1 : -1;
        
        // Adjust direction based on behavior
        if (directionBehavior == DirectionBehavior.FORWARD) {
            direction = 1;
        } else if (directionBehavior == DirectionBehavior.BACKWARD) {
            direction = -1;
        } else if (directionBehavior == DirectionBehavior.RANDOM && 
                   (cycle < 0.01 || Math.abs(cycle - 0.5) < 0.01)) {
            direction = random.nextBoolean() ? 1 : -1;
        }
        
        // Reset brightness array
        Arrays.fill(brightness, 0.0);
        
        // Apply eye and tail brightness
        applyEyeAndTail(position, direction);
    }
    
    private void updateDualScan(double dt, int length) {
        // Dual scan: two scanners moving in opposite directions
        double moveAmount = dt * (length / cycleTime) * speed;
        position = (position + moveAmount) % length;
        
        // Reset brightness array
        Arrays.fill(brightness, 0.0);
        
        // Apply eye and tail for first scanner
        double pos1 = position;
        double dir1 = directionBehavior == DirectionBehavior.BACKWARD ? -1 : 1;
        applyEyeAndTail(pos1, dir1);
        
        // Apply eye and tail for second scanner
        double pos2 = (length - position) % length;
        double dir2 = directionBehavior == DirectionBehavior.FORWARD ? 1 : -1;
        applyEyeAndTail(pos2, dir2);
    }
    
    private void updateSplitScan(double dt, int length) {
        // Split scan: starts in middle and splits outward, then returns
        double totalDistance = length / 2;
        double moveAmount = dt * (totalDistance / cycleTime) * speed;
        
        // Keep track of position in the cycle (0 to 1)
        double cycle = (Timer.getFPGATimestamp() - startTime) % cycleTime / cycleTime;
        
        // Calculate position using triangle wave for splitting effect
        double normalizedPos;
        if (cycle < 0.5) {
            normalizedPos = cycle * 2; // Expanding outward
        } else {
            normalizedPos = 2 - cycle * 2; // Contracting inward
        }
        
        position = normalizedPos * (length / 2 - eyeSize);
        
        // Direction is the derivative of position
        direction = cycle < 0.5 ? 1 : -1;
        
        // Adjust direction based on behavior
        if (directionBehavior == DirectionBehavior.FORWARD) {
            direction = 1;
        } else if (directionBehavior == DirectionBehavior.BACKWARD) {
            direction = -1;
        } else if (directionBehavior == DirectionBehavior.RANDOM && 
                   (cycle < 0.01 || Math.abs(cycle - 0.5) < 0.01)) {
            direction = random.nextBoolean() ? 1 : -1;
        }
        
        // Reset brightness array
        Arrays.fill(brightness, 0.0);
        
        // Apply eye and tail for first scanner (left side)
        double center = length / 2;
        double pos1 = center - position;
        double dir1 = -direction;
        applyEyeAndTail(pos1, dir1);
        
        // Apply eye and tail for second scanner (right side)
        double pos2 = center + position;
        double dir2 = direction;
        applyEyeAndTail(pos2, dir2);
    }
    
    private void applyEyeAndTail(double pos, double dir) {
        int length = buffer.getLength();
        int eyePos = (int) Math.round(pos);
        
        // Apply eye (scanner)
        for (int i = 0; i < eyeSize; i++) {
            int pixelPos = eyePos + i;
            if (pixelPos >= 0 && pixelPos < length) {
                brightness[pixelPos] = Math.max(brightness[pixelPos], 1.0); // Full brightness
            }
        }
        
        // Apply tail
        if (tailLength > 0) {
            int tailStart = dir > 0 ? eyePos - tailLength : eyePos + eyeSize;
            int tailEnd = dir > 0 ? eyePos : eyePos + eyeSize + tailLength;
            
            for (int i = tailStart; i < tailEnd; i++) {
                if (i >= 0 && i < length && i != eyePos) {
                    // Calculate distance from eye as a percentage of tail length
                    double distance = dir > 0 ? eyePos - i : i - (eyePos + eyeSize - 1);
                    double normalizedDistance = Math.max(0, Math.min(1, distance / tailLength));
                    
                    // Calculate brightness based on tail style
                    double tailBright = 0;
                    switch (tailStyle) {
                        case FADE_OUT:
                            tailBright = 1.0 - normalizedDistance;
                            break;
                        case SHARP_CUT:
                            tailBright = normalizedDistance < 0.5 ? 1.0 : 0.0;
                            break;
                        case PULSE:
                            tailBright = Math.max(0, Math.cos(normalizedDistance * Math.PI * 2));
                            break;
                        case GLOW:
                            tailBright = Math.max(0, Math.cos(normalizedDistance * Math.PI / 2));
                            break;
                        case EXPONENTIAL:
                            tailBright = Math.exp(-3 * normalizedDistance);
                            break;
                    }
                    
                    // Apply tail brightness factor
                    tailBright *= tailBrightness;
                    
                    // Set the brightness (max with existing brightness)
                    brightness[i] = Math.max(brightness[i], tailBright);
                }
            }
        }
    }
    
    private void applyBrightnessToBuffer() {
        for (int i = 0; i < buffer.getLength(); i++) {
            double bright = brightness[i];
            if (bright > 0) {
                if (rainbowType == RainbowType.NONE) {
                    // Use the specified color
                    int r = (int) (color.red * 255 * bright);
                    int g = (int) (color.green * 255 * bright);
                    int b = (int) (color.blue * 255 * bright);
                    buffer.setRGB(i, r, g, b);
                } else {
                    // Use the rainbow color
                    // Calculate hue based on position or time
                    double hue;
                    switch (rainbowType) {
                        case RGB_RAINBOW:
                            hue = (i / (double) buffer.getLength() + rainbowPhase) % 1.0;
                            break;
                        case PASTEL_RAINBOW:
                            hue = (i / (double) buffer.getLength() + rainbowPhase) % 1.0;
                            break;
                        case FIRE:
                            // Fire colors (red to yellow only)
                            hue = ((i / (double) buffer.getLength() + rainbowPhase) % 1.0) * 0.1;
                            break;
                        case OCEAN:
                            // Ocean colors (blue to green only)
                            hue = 0.5 + ((i / (double) buffer.getLength() + rainbowPhase) % 1.0) * 0.2;
                            break;
                        default:
                            hue = 0;
                            break;
                    }
                    
                    RGB rgb = getRainbowColor(hue, rainbowType);
                    int r = (int) (rgb.r * bright);
                    int g = (int) (rgb.g * bright);
                    int b = (int) (rgb.b * bright);
                    buffer.setRGB(i, r, g, b);
                }
            } else {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
    }
    
    // Get a rainbow color based on hue (0.0 to 1.0)
    private RGB getRainbowColor(double hue, RainbowType type) {
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
        
        // Apply additional color adjustments based on type
        if (type == RainbowType.PASTEL_RAINBOW) {
            // Pastels are created by mixing with white
            r = r + (255 - r) / 2;
            g = g + (255 - g) / 2;
            b = b + (255 - b) / 2;
        } else if (type == RainbowType.FIRE) {
            // Fire colors emphasize red and yellow
            r = Math.min(255, r * 2);
            g = (int)(g * 0.7);
            b = Math.min(40, b);
        } else if (type == RainbowType.OCEAN) {
            // Ocean colors emphasize blue and cyan
            r = Math.min(30, r);
            g = (int)(g * 0.8);
            b = Math.min(255, b * 2);
        }
        
        return new RGB(r, g, b);
    }
    
    @Override
    public void end() {
        // Clear the LED buffer when animation ends
        if (buffer != null) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
    }
}
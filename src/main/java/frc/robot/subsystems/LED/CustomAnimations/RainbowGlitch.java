package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class RainbowGlitch implements LEDAnimation {
    // Rainbow type enum
    public enum RainbowType {
        RGB_RAINBOW,  // Standard bright rainbow colors
        PASTEL_RAINBOW // Softer pastel rainbow colors
    }
    
    // Color distribution enum
    public enum ColorDistribution {
        PER_SEGMENT, // Each segment gets a different rainbow color
        PER_LED      // Each individual LED in a segment gets its own rainbow color
    }
    
    private final RainbowType rainbowType;
    private final ColorDistribution colorDistribution;
    private final int segmentSize;
    private final double cycleTime;
    private final double rainbowCycleTime; // Time for rainbow colors to cycle
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private List<GlitchSegment> activeSegments = new ArrayList<>();
    private double cycleStartTime;
    private double lastUpdateTime;
    private final Random random = new Random();
    
    /**
     * Creates a rainbow glitching LED animation with chaotic per-LED behavior in segments.
     *
     * @param rainbowType      The type of rainbow colors to use
     * @param colorDistribution Whether to apply rainbow colors per segment or per LED
     * @param segmentSize      The number of LEDs per glitched segment. Use 0 for full strip.
     * @param cycleTime        The time in seconds for the entire glitch cycle to restart
     * @param rainbowCycleTime The time in seconds for rainbow colors to cycle
     */
    public RainbowGlitch(RainbowType rainbowType, ColorDistribution colorDistribution, 
                        int segmentSize, double cycleTime, double rainbowCycleTime) {
        this.rainbowType = rainbowType;
        this.colorDistribution = colorDistribution;
        this.segmentSize = segmentSize; // Allow 0 for full strip
        this.cycleTime = Math.max(0.1, cycleTime);
        this.rainbowCycleTime = Math.max(0.5, rainbowCycleTime);
    }
    
    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
        this.lastUpdateTime = Timer.getFPGATimestamp();
        this.cycleStartTime = this.lastUpdateTime;
        
        // Generate initial set of random segments
        generateRandomSegments();
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        
        // Check if it's time for a new cycle
        if (currentTime - cycleStartTime > cycleTime) {
            cycleStartTime = currentTime;
            generateRandomSegments();
        }
        
        // Clear the LED buffer
        clearBuffer();
        
        // Calculate the current rainbow phase (0.0 to 1.0) based on the cycle time
        double rainbowPhase = (currentTime % rainbowCycleTime) / rainbowCycleTime;
        
        // Update and render all active segments
        for (GlitchSegment segment : activeSegments) {
            segment.update(dt, currentTime, rainbowPhase);
            segment.render(buffer);
        }
    }
    
    @Override
    public void end() {
        // Clear the LED buffer when animation ends
        if (buffer != null) {
            clearBuffer();
        }
        activeSegments.clear();
    }
    
    // Generate a new set of random segments or a full strip segment
    private void generateRandomSegments() {
        activeSegments.clear();
        
        int stripLength = buffer.getLength();
        
        // Special case: If segmentSize is 0, create one segment for the entire strip
        if (segmentSize == 0) {
            double rainbowOffset = 0.0; // Start at beginning of rainbow
            activeSegments.add(new GlitchSegment(0, stripLength, rainbowOffset));
            return;
        }
        
        // Normal case: Create multiple random segments
        int totalSegments = stripLength / segmentSize;
        
        // Target about half the segments to be active
        int targetActive = Math.max(1, totalSegments / 2);
        
        for (int i = 0; i < targetActive; i++) {
            // Generate a random start position that doesn't go off the end of the strip
            int startPos = random.nextInt(stripLength - segmentSize + 1);
            
            // Create and add new segment with a rainbow offset based on position
            double rainbowOffset = (double) startPos / stripLength;
            activeSegments.add(new GlitchSegment(startPos, segmentSize, rainbowOffset));
        }
    }
    
    // Clear all LEDs in the buffer
    private void clearBuffer() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
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
    
    // Represents a single glitching LED
    private class GlitchLED {
        private final int index;
        private double brightness = 0.0;
        private final double rainbowOffset; // Phase offset for rainbow color
        
        // Each LED gets its own randomized parameters
        private final double blinkSpeed; // Blinks per second
        private final double minBrightness;
        private final double maxBrightness;
        private final double flickerChance; // Chance to flicker per update
        private final int behaviorType; // Different behavior patterns
        private double timeSinceStateChange = 0.0; // For blink timing
        private boolean blinkState = true; // For toggling on/off
        
        public GlitchLED(int index, double rainbowOffset) {
            this.index = index;
            this.rainbowOffset = rainbowOffset;
            
            // Randomize LED behavior
            this.blinkSpeed = 1.0 + random.nextDouble() * 15.0; // 1-16 blinks per second
            this.minBrightness = 0.05 + random.nextDouble() * 0.35; // 5-40% min brightness (never fully off)
            this.maxBrightness = 0.6 + random.nextDouble() * 0.4; // 60-100% max brightness
            this.flickerChance = 0.1 + random.nextDouble() * 0.3; // 10-40% chance
            this.behaviorType = random.nextInt(4); // 4 different behavior patterns
            
            // Always start with some brightness (never off)
            brightness = minBrightness + random.nextDouble() * (maxBrightness - minBrightness);
        }
        
        // Update LED state
        public void update(double dt, double timestamp) {
            // Update blink timing
            timeSinceStateChange += dt;
            double blinkDuration = 1.0 / blinkSpeed;
            
            // Each behavior type has a unique pattern
            switch (behaviorType) {
                case 0: // Regular blinking
                    if (timeSinceStateChange > blinkDuration) {
                        blinkState = !blinkState;
                        timeSinceStateChange = 0;
                    }
                    brightness = blinkState ? maxBrightness : minBrightness;
                    break;
                    
                case 1: // Sine wave pulsing
                    // Unique phase for this LED based on its index and current time
                    double phase = (timestamp * blinkSpeed + index * 0.5) % 1.0;
                    double pulseAmount = (Math.sin(phase * 2 * Math.PI) + 1) / 2;
                    brightness = minBrightness + pulseAmount * (maxBrightness - minBrightness);
                    break;
                    
                case 2: // Sawtooth pattern (rapid rise, slow fall)
                    double sawPhase = (timestamp * blinkSpeed) % 1.0;
                    if (sawPhase < 0.1) {
                        // Quick rise (10% of cycle)
                        brightness = minBrightness + (sawPhase / 0.1) * (maxBrightness - minBrightness);
                    } else {
                        // Slow fall (90% of cycle)
                        brightness = maxBrightness - ((sawPhase - 0.1) / 0.9) * (maxBrightness - minBrightness);
                    }
                    break;
                    
                case 3: // Erratic flickering
                    if (random.nextDouble() < flickerChance) {
                        // Randomly jump to a new brightness (but never below minimum)
                        brightness = minBrightness + random.nextDouble() * (maxBrightness - minBrightness);
                    }
                    break;
            }
            
            // Random extra flicker (but never below minimum brightness)
            if (random.nextDouble() < flickerChance * 0.5) {
                brightness = Math.max(minBrightness, brightness * (random.nextDouble() * 0.5 + 0.5)); // 50-100% of current brightness, but never below minimum
            }
        }
        
        // Render this LED to the buffer
        public void render(AddressableLEDBuffer buffer, RGB segmentColor, double currentRainbowPhase) {
            if (index >= 0 && index < buffer.getLength()) {
                RGB colorToUse;
                
                // Determine which color to use based on configuration
                if (colorDistribution == ColorDistribution.PER_LED) {
                    // Each LED gets its own unique rainbow color
                    // Fixed: Use a combination of the LED's index and the current phase to get a unique color
                    // The formula creates a unique distribution pattern across the LEDs
                    double individualHue = (currentRainbowPhase + rainbowOffset + (index * 0.03)) % 1.0;
                    colorToUse = getRainbowColor(individualHue);
                } else {
                    // Use the segment's color
                    colorToUse = segmentColor;
                }
                
                // Adjust color based on brightness
                int r = (int)(colorToUse.r * brightness);
                int g = (int)(colorToUse.g * brightness);
                int b = (int)(colorToUse.b * brightness);
                
                buffer.setRGB(index, r, g, b);
            }
        }
    }
    
    // Represents a segment of glitching LEDs
    private class GlitchSegment {
        private final int startIndex;
        private final int length;
        private final double rainbowOffset;
        private final List<GlitchLED> leds = new ArrayList<>();
        
        // Segment color will be based on rainbow
        private RGB segmentColor;
        
        public GlitchSegment(int startIndex, int length, double rainbowOffset) {
            this.startIndex = startIndex;
            this.length = length;
            this.rainbowOffset = rainbowOffset;
            
            // The segment color will be set during update based on the current rainbow phase
            this.segmentColor = getRainbowColor(rainbowOffset);
            
            // Create individual LEDs for this segment
            for (int i = 0; i < length; i++) {
                // Fixed: Give each LED a unique rainbowOffset based on its position
                // This ensures diversity of colors when using PER_LED distribution
                double ledOffset;
                if (colorDistribution == ColorDistribution.PER_LED) {
                    // Distribute unique offsets for each LED
                    // Using a wider factor for more distinct color variation
                    ledOffset = rainbowOffset + ((double)i / length) * 0.7;
                } else {
                    // For PER_SEGMENT, all LEDs in segment share same offset
                    ledOffset = rainbowOffset;
                }
                leds.add(new GlitchLED(startIndex + i, ledOffset));
            }
        }
        
        // Update all LEDs in this segment
        public void update(double dt, double timestamp, double rainbowPhase) {
            // Update segment color based on current rainbow phase
            // Only need to do this for PER_SEGMENT distribution
            if (colorDistribution == ColorDistribution.PER_SEGMENT) {
                double segmentHue = (rainbowPhase + rainbowOffset) % 1.0;
                segmentColor = getRainbowColor(segmentHue);
                
                // Add some variation to segment color
                int rVar = (int)(random.nextDouble() * 40) - 20; // +/- 20
                int gVar = (int)(random.nextDouble() * 40) - 20;
                int bVar = (int)(random.nextDouble() * 40) - 20;
                
                segmentColor = new RGB(
                    Math.max(0, Math.min(255, segmentColor.r + rVar)),
                    Math.max(0, Math.min(255, segmentColor.g + gVar)),
                    Math.max(0, Math.min(255, segmentColor.b + bVar))
                );
            }
            
            // Update individual LEDs
            for (GlitchLED led : leds) {
                led.update(dt, timestamp);
            }
        }
        
        // Render all LEDs in this segment
        public void render(AddressableLEDBuffer buffer) {
            // Current rainbow phase is passed to LEDs for per-LED coloring
            double currentRainbowPhase = (Timer.getFPGATimestamp() % rainbowCycleTime) / rainbowCycleTime;
            
            for (GlitchLED led : leds) {
                led.render(buffer, segmentColor, currentRainbowPhase);
            }
        }
    }
}
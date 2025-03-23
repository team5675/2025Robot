package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Glitch implements LEDAnimation {
    private final RGB glitchColor;
    private final int segmentSize;
    private final double cycleTime;
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private List<GlitchSegment> activeSegments = new ArrayList<>();
    private double cycleStartTime;
    private double lastUpdateTime;
    private final Random random = new Random();
    
    /**
     * Creates a glitching LED animation with chaotic per-LED behavior in segments.
     *
     * @param glitchColor The RGB color for the glitching segments
     * @param segmentSize The number of LEDs per glitched segment. Use 0 for full strip.
     * @param cycleTime   The time in seconds for the entire glitch cycle to restart
     */
    public Glitch(RGB glitchColor, int segmentSize, double cycleTime) {
        this.glitchColor = glitchColor;
        this.segmentSize = segmentSize; // Allow 0 for full strip
        this.cycleTime = Math.max(0.1, cycleTime);
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
        
        // Update and render all active segments
        for (GlitchSegment segment : activeSegments) {
            segment.update(dt, currentTime);
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
            activeSegments.add(new GlitchSegment(0, stripLength));
            return;
        }
        
        // Normal case: Create multiple random segments
        int totalSegments = stripLength / segmentSize;
        
        // Target about half the segments to be active
        int targetActive = Math.max(1, totalSegments / 2);
        
        for (int i = 0; i < targetActive; i++) {
            // Generate a random start position that doesn't go off the end of the strip
            int startPos = random.nextInt(stripLength - segmentSize + 1);
            
            // Create and add new segment
            activeSegments.add(new GlitchSegment(startPos, segmentSize));
        }
    }
    
    // Clear all LEDs in the buffer
    private void clearBuffer() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }
    
    // Represents a single glitching LED
    private class GlitchLED {
        private final int index;
        private double brightness = 0.0;
        
        // Each LED gets its own randomized parameters
        private final double blinkSpeed; // Blinks per second
        private final double minBrightness;
        private final double maxBrightness;
        private final double flickerChance; // Chance to flicker per update
        private final int behaviorType; // Different behavior patterns
        private double timeSinceStateChange = 0.0; // For blink timing
        private boolean blinkState = true; // For toggling on/off
        
        public GlitchLED(int index) {
            this.index = index;
            
            // Randomize LED behavior
            this.blinkSpeed = 1.0 + random.nextDouble() * 15.0; // 1-16 blinks per second
            this.minBrightness = 0.05 + random.nextDouble() * 0.35; // 5-40% min brightness (never fully off)
            this.maxBrightness = 0.6 + random.nextDouble() * 0.4; // 0.6-1.0
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
        public void render(AddressableLEDBuffer buffer, RGB color) {
            if (index >= 0 && index < buffer.getLength()) {
                // Adjust color based on brightness
                int r = (int)(color.r * brightness);
                int g = (int)(color.g * brightness);
                int b = (int)(color.b * brightness);
                
                buffer.setRGB(index, r, g, b);
            }
        }
    }
    
    // Represents a segment of glitching LEDs
    private class GlitchSegment {
        private final int startIndex;
        private final int length;
        private final List<GlitchLED> leds = new ArrayList<>();
        
        // Each segment can have its own color variation
        private final RGB segmentColor;
        
        public GlitchSegment(int startIndex, int length) {
            this.startIndex = startIndex;
            this.length = length;
            
            // Create slightly varied color for this segment
            int rVar = (int)(random.nextDouble() * 40) - 20; // +/- 20
            int gVar = (int)(random.nextDouble() * 40) - 20;
            int bVar = (int)(random.nextDouble() * 40) - 20;
            
            this.segmentColor = new RGB(
                Math.max(0, Math.min(255, glitchColor.r + rVar)),
                Math.max(0, Math.min(255, glitchColor.g + gVar)),
                Math.max(0, Math.min(255, glitchColor.b + bVar))
            );
            
            // Create individual LEDs for this segment
            for (int i = 0; i < length; i++) {
                leds.add(new GlitchLED(startIndex + i));
            }
        }
        
        // Update all LEDs in this segment
        public void update(double dt, double timestamp) {
            for (GlitchLED led : leds) {
                led.update(dt, timestamp);
            }
        }
        
        // Render all LEDs in this segment
        public void render(AddressableLEDBuffer buffer) {
            for (GlitchLED led : leds) {
                led.render(buffer, segmentColor);
            }
        }
    }
}
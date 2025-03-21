package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

public class RainbowShootingLines implements LEDAnimation {
    // Rainbow type enum
    public enum RainbowType {
        RGB_RAINBOW,  // Standard bright rainbow colors
        PASTEL_RAINBOW // Softer pastel rainbow colors
    }
    
    // Color distribution enum
    public enum ColorDistribution {
        PER_LINE,    // Each line gets a different rainbow color
        CYCLE_RAINBOW // All lines cycle through rainbow colors together
    }
    
    // Direction type enum
    public enum DirectionType {
        FORWARD,  // Lines move in the forward direction
        BACKWARD, // Lines move in the backward direction
        RANDOM    // Each line gets a random direction
    }
    
    private final RainbowType rainbowType;
    private final ColorDistribution colorDistribution;
    private final DirectionType directionType; // Direction type
    private final int lineLength; // Number of LEDs for a line to reach min brightness
    private final double minBrightness;
    private final double spawnInterval; // Time in seconds between spawning new lines
    private final double speed; // Speed factor (0.0 to 1.0), 0 means random speed per line
    private final double rainbowCycleTime; // Time in seconds for rainbow colors to cycle
    private final boolean quickInitialize; // Whether to initialize with lines already on the strip
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private double lastSpawnTime;
    private double lastUpdateTime;
    private List<RainbowLine> activeLines = new ArrayList<>();
    private boolean waitingForLineTail = false;
    private final Random random = new Random();
    
    /**
     * Creates an animation with rainbow-colored lines that shoot across the LED strip
     * 
     * @param rainbowType The type of rainbow colors to use
     * @param colorDistribution Whether each line gets its own color or all lines cycle together
     * @param directionType Direction for lines to move (FORWARD, BACKWARD, or RANDOM)
     * @param lineLength The number of LEDs for a line to fade out
     * @param minBrightness Minimum brightness (0.0 to 1.0) at the tail of each line
     * @param spawnInterval Time in seconds between each shooting line spawning (0 to wait for line tail)
     * @param speed Speed factor (0.0 to 1.0) controlling how fast lines move, 0 means random speed per line
     * @param rainbowCycleTime Time in seconds for rainbow colors to complete a full cycle
     * @param quickInitialize Whether to initialize with lines already on the strip
     */
    public RainbowShootingLines(RainbowType rainbowType, ColorDistribution colorDistribution,
                              DirectionType directionType, int lineLength, double minBrightness, 
                              double spawnInterval, double speed, 
                              double rainbowCycleTime,
                              boolean quickInitialize) {
        this.rainbowType = rainbowType;
        this.colorDistribution = colorDistribution;
        this.directionType = directionType;
        this.lineLength = Math.max(1, lineLength); // Ensure line length is at least 1
        this.minBrightness = minBrightness;
        this.spawnInterval = spawnInterval;
        this.speed = speed; // Can be 0 now for random speed per line
        this.rainbowCycleTime = Math.max(0.5, rainbowCycleTime);
        this.quickInitialize = quickInitialize;
    }
    
    // Backward compatibility constructor
    public RainbowShootingLines(RainbowType rainbowType, ColorDistribution colorDistribution,
                              int lineLength, double minBrightness, 
                              double spawnInterval, double speed, 
                              double rainbowCycleTime,
                              boolean quickInitialize, boolean forward) {
        this(
            rainbowType,
            colorDistribution,
            forward ? DirectionType.FORWARD : DirectionType.BACKWARD,
            lineLength,
            minBrightness,
            spawnInterval,
            speed,
            rainbowCycleTime,
            quickInitialize
        );
    }
    
    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.buffer = ledSubsystem.getBuffer();
        this.lastSpawnTime = Timer.getFPGATimestamp();
        this.lastUpdateTime = lastSpawnTime;
        this.activeLines.clear();
        
        // Initialize with lines already on the strip if requested
        if (quickInitialize) {
            initializeWithLines();
        }
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        
        // Clear the LED buffer
        clearBuffer();
        
        // Check if it's time to spawn a new line
        boolean shouldSpawnNewLine = false;
        
        if (spawnInterval <= 0) {
            // Spawn mode: wait for line tail to enter
            if (!waitingForLineTail || activeLines.isEmpty()) {
                shouldSpawnNewLine = true;
                waitingForLineTail = true;
            } else if (waitingForLineTail && !activeLines.isEmpty()) {
                // Check if the most recently added line's tail has entered the strip
                RainbowLine lastLine = activeLines.get(activeLines.size() - 1);
                if (lastLine.tailHasEntered()) {
                    shouldSpawnNewLine = true;
                }
            }
        } else {
            // Spawn mode: time-based interval
            if (currentTime - lastSpawnTime >= spawnInterval) {
                shouldSpawnNewLine = true;
            }
        }
        
        if (shouldSpawnNewLine) {
            // Current rainbow phase for new line
            double rainbowPhase = (currentTime % rainbowCycleTime) / rainbowCycleTime;
            
            // Generate a random speed if speed is set to 0
            double lineSpeed = (speed <= 0) ? 
                0.5 + random.nextDouble() * 0.5 : // Random between 0.5 and 1.0
                speed;
                
            // Determine direction based on directionType
            boolean forward;
            switch (directionType) {
                case FORWARD:
                    forward = true;
                    break;
                case BACKWARD:
                    forward = false;
                    break;
                case RANDOM:
                default:
                    forward = random.nextBoolean();
                    break;
            }
            
            activeLines.add(new RainbowLine(rainbowPhase, lineSpeed, forward));
            lastSpawnTime = currentTime;
            if (spawnInterval <= 0) {
                waitingForLineTail = true;
            }
        }
        
        // Calculate the current rainbow phase for CYCLE_RAINBOW mode
        double currentRainbowPhase = (currentTime % rainbowCycleTime) / rainbowCycleTime;
        
        // Update and draw all active lines
        Iterator<RainbowLine> iterator = activeLines.iterator();
        while (iterator.hasNext()) {
            RainbowLine line = iterator.next();
            line.advance(dt);
            
            // Draw the line with appropriate rainbow coloring
            drawLine(line, currentRainbowPhase);
            
            // Remove completed lines
            if (line.isComplete()) {
                iterator.remove();
                if (activeLines.isEmpty() && spawnInterval <= 0) {
                    waitingForLineTail = false;
                }
            }
        }
    }
    
    @Override
    public void end() {
        // Clear the LED buffer when animation ends
        if (buffer != null) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        activeLines.clear();
    }
    
    // Initialize the strip with evenly spaced lines
    private void initializeWithLines() {
        int bufferLength = buffer.getLength();
        double lineSpacing;
        
        // If spawn interval is 0, position lines adjacent to each other
        if (spawnInterval <= 0) {
            // Place lines adjacent to each other (head to tail)
            lineSpacing = lineLength;
        } else {
            // Calculate how far a line would travel during the spawn interval
            // Use base speed if speed is 0 (will be randomized for each line later)
            double baseSpeed = (speed <= 0) ? 0.75 : speed; // Use middle value of random range
            lineSpacing = 60.0 * baseSpeed * spawnInterval;
            // Ensure spacing is at least the length of a line
            lineSpacing = Math.max(lineLength, lineSpacing);
        }
        
        // Create initial rainbow offset for first line
        double initialRainbowPhase = 0.0;
        double rainbowPhaseStep = 0.15; // Color offset between initial lines
        
        // Determine a consistent direction for initialization based on directionType
        boolean initForward;
        switch (directionType) {
            case FORWARD:
                initForward = true;
                break;
            case BACKWARD:
                initForward = false;
                break;
            case RANDOM:
            default:
                // For initialization with RANDOM direction, pick one consistent direction
                // Individual lines will get random directions when they are created
                initForward = true;
                break;
        }
        
        // Add lines at proper intervals
        if (initForward) {
            // Start from the right end and go leftward so the first line is furthest right
            for (double pos = bufferLength - 1; pos > -lineLength; pos -= lineSpacing) {
                // Generate random speed if needed
                double lineSpeed = (speed <= 0) ? 
                    0.5 + random.nextDouble() * 0.5 : // Random between 0.5 and 1.0
                    speed;
                
                // Generate random direction if needed
                boolean lineForward = directionType == DirectionType.RANDOM ? 
                    random.nextBoolean() : (directionType == DirectionType.FORWARD);
                
                RainbowLine line = new RainbowLine(initialRainbowPhase, lineSpeed, lineForward);
                line.position = pos;
                activeLines.add(0, line); // Add to beginning of list so order is preserved
                
                // Update rainbow phase for next line if using PER_LINE distribution
                if (colorDistribution == ColorDistribution.PER_LINE) {
                    initialRainbowPhase = (initialRainbowPhase + rainbowPhaseStep) % 1.0;
                }
            }
        } else {
            // Start from the left end and go rightward so the first line is furthest left
            for (double pos = 0; pos < bufferLength + lineLength; pos += lineSpacing) {
                // Generate random speed if needed
                double lineSpeed = (speed <= 0) ? 
                    0.5 + random.nextDouble() * 0.5 : // Random between 0.5 and 1.0
                    speed;
                
                // Generate random direction if needed
                boolean lineForward = directionType == DirectionType.RANDOM ? 
                    random.nextBoolean() : (directionType == DirectionType.FORWARD);
                
                RainbowLine line = new RainbowLine(initialRainbowPhase, lineSpeed, lineForward);
                line.position = pos;
                activeLines.add(0, line); // Add to beginning of list so order is preserved
                
                // Update rainbow phase for next line if using PER_LINE distribution
                if (colorDistribution == ColorDistribution.PER_LINE) {
                    initialRainbowPhase = (initialRainbowPhase + rainbowPhaseStep) % 1.0;
                }
            }
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
    
    // Represents a single rainbow shooting line
    private class RainbowLine {
        double position; // Head position of the line (using double for smoother movement)
        double rainbowPhase; // Fixed rainbow phase for this line (0.0 to 1.0)
        double lineSpeed; // Speed for this specific line
        boolean forward; // Direction for this specific line
        
        public RainbowLine(double rainbowPhase, double lineSpeed, boolean forward) {
            // Set initial position based on direction
            position = forward ? -1 : buffer.getLength();
            this.rainbowPhase = rainbowPhase;
            this.lineSpeed = lineSpeed;
            this.forward = forward;
        }
        
        // Returns true if the line is completely off the strip
        public boolean isComplete() {
            return forward ? (position - lineLength >= buffer.getLength()) 
                          : (position + lineLength < 0);
        }
        
        // Returns true if the tail of the line has entered the strip
        public boolean tailHasEntered() {
            return forward ? (position >= lineLength - 1) 
                          : (position <= buffer.getLength() - lineLength);
        }
        
        // Move the line forward or backward based on elapsed time and speed
        public void advance(double dt) {
            double moveAmount = 60.0 * lineSpeed * dt; // Base speed of 60 LEDs per second * speed factor
            position = forward ? position + moveAmount : position - moveAmount;
        }
    }
    
    // Clears all LEDs in the buffer
    private void clearBuffer() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }
    
    // Draws a single rainbow line to the LED buffer
    private void drawLine(RainbowLine line, double currentRainbowPhase) {
        int bufferLength = buffer.getLength();
        
        // Calculate the head position (integer part of the position)
        int headPos = (int) Math.round(line.position);
        
        // Calculate the range of LEDs that need to be updated for this line
        int startPos = line.forward ? Math.max(0, headPos - lineLength + 1) : Math.min(bufferLength - 1, headPos + lineLength - 1);
        int endPos = line.forward ? Math.min(bufferLength - 1, headPos) : Math.max(0, headPos);
        
        // Only draw if some part of the line is visible
        if ((line.forward && startPos <= endPos) || (!line.forward && startPos >= endPos)) {
            // Determine which rainbow phase to use
            double phaseToUse;
            if (colorDistribution == ColorDistribution.PER_LINE) {
                // Each line keeps its assigned rainbow phase
                phaseToUse = line.rainbowPhase;
            } else {
                // All lines cycle through rainbow colors together
                phaseToUse = currentRainbowPhase;
            }
            
            // Get the color for this line
            RGB lineColor = getRainbowColor(phaseToUse);
            
            // Iterate through the relevant LEDs
            for (int i = startPos; line.forward ? i <= endPos : i >= endPos; i = line.forward ? i + 1 : i - 1) {
                // Calculate the distance from the head of the line
                int distance = line.forward ? endPos - i : i - endPos;
                
                // Calculate brightness based on distance
                double fadeFactor = 1.0 - ((1.0 - minBrightness) * distance / (lineLength - 1));
                // Ensure brightness doesn't go below minBrightness
                fadeFactor = Math.max(minBrightness, fadeFactor);
                
                // Set the LED color with adjusted brightness
                int adjustedRed = (int)(lineColor.r * fadeFactor);
                int adjustedGreen = (int)(lineColor.g * fadeFactor);
                int adjustedBlue = (int)(lineColor.b * fadeFactor);
                
                buffer.setRGB(i, adjustedRed, adjustedGreen, adjustedBlue);
            }
        }
    }
}
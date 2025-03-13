package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class ShootingLines implements LEDAnimation {
    private final RGB color;
    private final boolean forward; // Direction: true = forward, false = backward
    private final int lineLength; // Number of LEDs for a line to reach min brightness
    private final double minBrightness;
    private final double spawnInterval; // Time in seconds between spawning new lines
    private final double speed; // Speed factor (0.0 to 1.0)
    private final boolean quickInitialize; // Whether to initialize with lines already on the strip
    
    private LED ledSubsystem;
    private AddressableLEDBuffer buffer;
    private double lastSpawnTime;
    private double lastUpdateTime;
    private List<Line> activeLines = new ArrayList<>();
    private boolean waitingForLineTail = false;
    
    /**
     * Creates an animation with lines that shoot across the LED strip
     * 
     * @param color The RGB color of the shooting lines
     * @param forward Direction (true = forward from 0 to end, false = backward from end to 0)
     * @param lineLength The number of LEDs for a line to fade out
     * @param minBrightness Minimum brightness (0.0 to 1.0) at the tail of each line
     * @param spawnInterval Time in seconds between each shooting line spawning (0 to wait for line tail)
     * @param speed Speed factor (0.0 to 1.0) controlling how fast lines move
     * @param quickInitialize Whether to initialize with lines already on the strip
     */
    public ShootingLines(RGB color, boolean forward, int lineLength, double minBrightness, 
                         double spawnInterval, double speed, boolean quickInitialize) {
        this.color = color;
        this.forward = forward;
        this.lineLength = Math.max(1, lineLength); // Ensure line length is at least 1
        this.minBrightness = minBrightness;
        this.spawnInterval = spawnInterval;
        this.speed = Math.max(0.1, Math.min(1.0, speed)); // Clamp speed between 0.1 and 1.0
        this.quickInitialize = quickInitialize;
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
                Line lastLine = activeLines.get(activeLines.size() - 1);
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
            activeLines.add(new Line());
            lastSpawnTime = currentTime;
            if (spawnInterval <= 0) {
                waitingForLineTail = true;
            }
        }
        
        // Update and draw all active lines
        Iterator<Line> iterator = activeLines.iterator();
        while (iterator.hasNext()) {
            Line line = iterator.next();
            line.advance(dt);
            
            // Draw the line
            drawLine(line);
            
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
            lineSpacing = 60.0 * speed * spawnInterval;
            // Ensure spacing is at least the length of a line
            lineSpacing = Math.max(lineLength, lineSpacing);
        }
        
        // Add lines at proper intervals
        if (forward) {
            // Start from the right end and go leftward so the first line is furthest right
            for (double pos = bufferLength - 1; pos > -lineLength; pos -= lineSpacing) {
                Line line = new Line();
                line.position = pos;
                activeLines.add(0, line); // Add to beginning of list so order is preserved
            }
        } else {
            // Start from the left end and go rightward so the first line is furthest left
            for (double pos = 0; pos < bufferLength + lineLength; pos += lineSpacing) {
                Line line = new Line();
                line.position = pos;
                activeLines.add(0, line); // Add to beginning of list so order is preserved
            }
        }
    }
    
    // Represents a single shooting line
    private class Line {
        double position; // Head position of the line (using double for smoother movement)
        
        public Line() {
            // Set initial position based on direction
            position = forward ? -1 : buffer.getLength();
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
            double moveAmount = 60.0 * speed * dt; // Base speed of 60 LEDs per second * speed factor
            position = forward ? position + moveAmount : position - moveAmount;
        }
    }
    
    // Clears all LEDs in the buffer
    private void clearBuffer() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }
    
    // Draws a single line to the LED buffer
    private void drawLine(Line line) {
        int bufferLength = buffer.getLength();
        
        // Calculate the head position (integer part of the position)
        int headPos = (int) Math.round(line.position);
        
        // Calculate the range of LEDs that need to be updated for this line
        int startPos = forward ? Math.max(0, headPos - lineLength + 1) : Math.min(bufferLength - 1, headPos + lineLength - 1);
        int endPos = forward ? Math.min(bufferLength - 1, headPos) : Math.max(0, headPos);
        
        // Only draw if some part of the line is visible
        if ((forward && startPos <= endPos) || (!forward && startPos >= endPos)) {
            // Iterate through the relevant LEDs
            for (int i = startPos; forward ? i <= endPos : i >= endPos; i = forward ? i + 1 : i - 1) {
                // Calculate the distance from the head of the line
                int distance = forward ? endPos - i : i - endPos;
                
                // Calculate brightness based on distance
                double fadeFactor = 1.0 - ((1.0 - minBrightness) * distance / (lineLength - 1));
                // Ensure brightness doesn't go below minBrightness
                fadeFactor = Math.max(minBrightness, fadeFactor);
                
                // Set the LED color with adjusted brightness
                int adjustedRed = (int)(color.r * fadeFactor);
                int adjustedGreen = (int)(color.g * fadeFactor);
                int adjustedBlue = (int)(color.b * fadeFactor);
                
                buffer.setRGB(i, adjustedRed, adjustedGreen, adjustedBlue);
            }
        }
    }
}
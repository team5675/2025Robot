package frc.robot.subsystems.LED.CustomAnimations;

import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;
import frc.robot.subsystems.LED.RGB;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class ElevatorAnimation implements LEDAnimation {
    private final RGB color;
    private final boolean fadeEffect;
    private LED ledSubsystem;

    public ElevatorAnimation(RGB color, boolean fadeEffect) {
        this.color = color;
        this.fadeEffect = fadeEffect;
    }

    @Override
    public void init(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void execute() {
        if (ledSubsystem == null) return;
        AddressableLEDBuffer buffer = ledSubsystem.getBuffer();
        int ledCount = ledSubsystem.getLength();
        
        int elevatorLevel = (int) Elevator.getInstance().ticksEncoder.getPosition();
        int ledIndex = Math.min(ledCount - 1, Math.max(0, elevatorLevel));
        
        for (int i = 0; i < ledCount; i++) {
            if (i <= ledIndex) {
                if (fadeEffect) {
                    double fadeFactor = 1.0 - ((double) (ledIndex - i) / ledCount);
                    buffer.setRGB(i, (int) (color.r * fadeFactor), (int) (color.g * fadeFactor), (int) (color.b * fadeFactor));
                } else {
                    buffer.setRGB(i, color.r, color.g, color.b);
                }
            } else {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    @Override
    public void end() {
        AddressableLEDBuffer buffer = ledSubsystem.getBuffer();
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }
}
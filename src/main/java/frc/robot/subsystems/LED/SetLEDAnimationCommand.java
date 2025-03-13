package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDAnimation;

public class SetLEDAnimationCommand extends Command {
    private final LED ledSubsystem;
    private final LEDAnimation animation;
    
    public SetLEDAnimationCommand(LEDAnimation animation) {
        this.ledSubsystem = LED.getInstance();
        this.animation = animation;
        addRequirements(ledSubsystem);
    }
    
    @Override
    public void initialize() {
        ledSubsystem.setAnimation(animation);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
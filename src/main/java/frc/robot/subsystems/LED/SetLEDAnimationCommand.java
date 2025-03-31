package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

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
        try {
            ledSubsystem.setAnimation(animation);
        } catch (Exception e) {
            // System.out.println("");
        }
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
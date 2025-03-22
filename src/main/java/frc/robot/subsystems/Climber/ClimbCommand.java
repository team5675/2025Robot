package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LEDStateManager;
import frc.robot.subsystems.LED.RGB;
import frc.robot.subsystems.LED.SetLEDAnimationCommand;
import frc.robot.subsystems.LED.CustomAnimations.Glitch;
import frc.robot.subsystems.LED.CustomAnimations.RainbowPulse;

public class ClimbCommand extends Command {
    private final Climber climber;

    public ClimbCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber); // Locks Climber subsystem
    }

    @Override
    public void initialize() {
        // System.out.println("Climbing Up...");
        climber.climberMotor.set(0.6);
    }

    
    // @Override
    // public void execute() {
    //     if(!climber.isTripped.getAsBoolean()){
    //         climber.climberMotor.set(0);
    //     }
    // }
    @Override
    public boolean isFinished() {
        return !climber.isClimbLimitSwitchTripped.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        climber.climberMotor.set(0);
        // System.out.println("Climb Complete.");

        new SetLEDAnimationCommand(
            LEDStateManager.getInstance().STARTING_SHOOTING_LINES
        ).schedule();

        //To Stay up in the air if needed
        //climber.climberMotor.set(0.1);
    }
}

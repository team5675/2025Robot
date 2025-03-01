package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
    private final Climber climber;

    public ClimbCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber); // Locks Climber subsystem
    }

    @Override
    public void initialize() {
        System.out.println("Climbing Up...");
        climber.SetTarget(ClimberConstants.climberticks);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climber.climberEncoder.getPosition() - ClimberConstants.climberticks) < 2; // Adjust tolerance
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Climb Complete.");
        //To Stay up in the air if needed
        //climber.climberMotor.set(0.1);
    }
}

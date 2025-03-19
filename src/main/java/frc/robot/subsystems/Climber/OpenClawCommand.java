package frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class OpenClawCommand extends Command {
    private final Climber climber;
    
    public OpenClawCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber); // Locks Climber subsystem
    }

    @Override
    public void initialize() {
        climber.clawMotor.setVoltage(ClimberConstants.openClaw);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}

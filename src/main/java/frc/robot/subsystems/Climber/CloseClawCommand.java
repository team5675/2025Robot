package frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class CloseClawCommand extends Command {
    private final Climber climber;
    
    public CloseClawCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber); // Locks Climber subsystem
    }

    @Override
    public void initialize() {
        // System.out.println("Closing Claw...");
        climber.clawMotor.setVoltage(ClimberConstants.closeClaw);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("Claw Closed.");
    }
}

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetClimbCommand extends Command {
    private final Climber climber;
    private final Timer timer = new Timer();
    private static final double TIMEOUT = 3.0; // Failsafe timeout in seconds

    public SetClimbCommand() {
        this.climber = Climber.getInstance();
        addRequirements(climber); // Ensures no other command interferes
    }

    @Override
    public void initialize() {
        System.out.println("Set Climb Command Ready");
        climber.clawMotor.setVoltage(0);
        climber.climberEncoder.setPosition(0);
        climber.SetTarget(ClimberConstants.setclimberticks);
        timer.reset();
        timer.start(); // Start the timeout timer
    }

    @Override
    public void execute() {

        if (climber.climberEncoder.getPosition() < ClimberConstants.setclimberticks) {
            climber.clawMotor.setVoltage(-3.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Set Climb Command Finished");
        climber.climberMotor.set(0);
        climber.clawMotor.setVoltage(0);
        climber.climberEncoder.setPosition(0); // Reset encoder position when finished
    }

    @Override
    public boolean isFinished() {
        // Ends when position is reached or timeout happens
        return Math.abs(climber.climberEncoder.getPosition() - ClimberConstants.setclimberticks) < 2 
                || timer.hasElapsed(TIMEOUT);
    }
}

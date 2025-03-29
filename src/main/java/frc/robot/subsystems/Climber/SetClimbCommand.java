package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.SetLEDAnimationCommand;
import frc.robot.subsystems.LED.CustomAnimations.RainbowPulse;

public class SetClimbCommand extends Command {
    private final Climber climber;

    public SetClimbCommand(Climber climber) {
        this.climber = Climber.getInstance();
        addRequirements(climber); // Ensures no other command interferes
    }

    @Override
    public void initialize() {
        // System.out.println("Set Climb Command Ready");
        //climber.clawMotor.setVoltage(0);
        climber.clawMotor.setVoltage(ClimberConstants.closeClaw);
        //climber.climberEncoder.setPosition(0);
        climber.climberMotor.set(-0.4);
        // climber.SetTarget(ClimberConstants.setclimberticks);

        new SetLEDAnimationCommand(
            new RainbowPulse(
                RainbowPulse.RainbowType.PASTEL_RAINBOW, 
                0, 
                1, 
                1.3,
                0
            )
        ).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        climber.climberMotor.set(0);
        climber.clawMotor.setVoltage(0);
        // climber.clawMotor.setVoltage(ClimberConstants.openClaw);
        // Commands.waitSeconds(1.5).andThen(() -> {
        //     climber.clawMotor.setVoltage(0);
        //     System.out.println("We waited 1.5 seconds");
        // }).schedule();

        // System.out.println("Set Climb Command Finished");
    }

    @Override
    public boolean isFinished() {
        // Ends when position is reached or timeout happens
        return !climber.isSetLimitSwitchTripped.getAsBoolean(); //|| Math.abs(climber.climberEncoder.getPosition() - ClimberConstants.setclimberticks) < 4;
    }
}

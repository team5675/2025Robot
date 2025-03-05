package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class UnClimbCommand extends Command{
    private final Climber climber;

public UnClimbCommand(Climber climber) {

    this.climber = Climber.getInstance();
    addRequirements(climber);
}
@Override
public void initialize(){
    System.out.println("UnClimb Command Ready" );
    climber.SetTarget(ClimberConstants.setclimberticks);
    climber.clawMotor.setVoltage(-3);
}
    
@Override
public void end(boolean interrupted){
    climber.clawMotor.setVoltage(0);
    climber.climberMotor.set(0);
}

@Override
public boolean isFinished() {
    return !climber.isSetLimitSwitchTripped.getAsBoolean();

}

}




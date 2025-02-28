package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class UnClimbCommand extends Command{
private Climber climber;

public UnClimbCommand() {

    climber = Climber.getInstance();
}
@Override
public void initialize(){
    System.out.println("UnClimb Command Ready" );
}
    
@Override
public void execute(){
    climber.climberMotor.set(-0.5);
    climber.clawMotor.setVoltage(-3);
}
@Override
public void end(boolean interrupted){
    climber.clawMotor.setVoltage(0);
    climber.climberMotor.set(0);
}

@Override
public boolean isFinished() {
    return false;

}

public static Object getInstance() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getInstance'");
}
}




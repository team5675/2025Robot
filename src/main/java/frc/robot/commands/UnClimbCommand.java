package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;

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
    climber.clawMotor.setVoltage(-5);
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




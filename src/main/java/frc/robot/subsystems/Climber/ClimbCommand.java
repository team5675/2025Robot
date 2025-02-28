package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimbCommand extends Command{
private Climber climber;
private final Timer clawtimer = new Timer();
private boolean isClimberMotorRunning;
public ClimbCommand() {
    climber = Climber.getInstance();
}

@Override
public void initialize(){
    System.out.println("Climb Command Ready" );
}
    
@Override
public void execute(){
    climber.isTripped.onFalse(null);
    if (!climber.isTripped.getAsBoolean()){
        //if (climber.clawMotor.getAppliedOutput() >=5.5)
        if(!isClimberMotorRunning) {
             climber.clawMotor.setVoltage(5.5); 
        climber.SetTarget(ClimberConstants.climberticks);
       
        
        }


}
}
@Override
public void end(boolean interrupted){
    climber.clawMotor.setVoltage(0);
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

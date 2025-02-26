
    package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SetClimbCommand extends Command{
private Climber climber;

public SetClimbCommand() {
    climber = Climber.getInstance();
}

@Override
public void initialize(){
    System.out.println("Set Climb Command Ready" );
}
    
@Override
public void execute(){
    if (climber.isTripped.getAsBoolean()){
        climber.clawMotor.setVoltage(-5);
    }
    else if (climber.isTripped.getAsBoolean()){
        climber.climberMotor.set(-0.5);
    }
    
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


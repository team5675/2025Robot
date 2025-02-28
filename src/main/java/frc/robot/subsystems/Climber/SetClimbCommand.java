
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
     climber.clawMotor.setVoltage(0);
     climber.climberEncoder.setPosition(0);
}
    
@Override
public void execute(){
    climber.SetTarget(ClimberConstants.setclimberticks);

    if (climber.climberEncoder.getPosition() < -42){
       climber.clawMotor.setVoltage(-3.5);
    }

    
    
}


@Override
public void end(boolean interrupted){
    climber.climberMotor.set(0);
    climber.clawMotor.setVoltage(0);
}

@Override
public boolean isFinished() {
    return false;

}
}


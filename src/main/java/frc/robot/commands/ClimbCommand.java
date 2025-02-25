package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber.Climber;

public class ClimbCommand extends Command{
private Climber climber;

public ClimbCommand() {
    climber = Climber.getInstance();
}

@Override
public void initialize(){
    System.out.println("Climb Command Ready" );
}
    
@Override
public void execute(){
    if (climber.isTripped.getAsBoolean()){
        climber.climberMotor.set(1);
        climber.clawMotor.setVoltage(0);
    }
else if (!climber.isTripped.getAsBoolean()){
    climber.clawMotor.setVoltage(5);
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

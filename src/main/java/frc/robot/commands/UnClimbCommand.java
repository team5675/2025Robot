package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber.Climber;

public class UnClimbCommand extends Command{
private Climber climber;

public UnClimbCommand() {
    climber = Climber.getInstance;
}
@Override
public void initialize(){
    System.out.println("UnClimb Command Ready" );
}
    
@Override
public void execute(){
    climber.clawmotor.setVoltage(-5);
}
@Override
public void end(boolean interrupted){
    climber.clawmotor.setVoltage(0);
    climber.ClimberMotor.set(0);
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




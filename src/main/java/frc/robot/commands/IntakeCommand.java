package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Coral.Coral;

public class IntakeCommand extends Command {

  private Coral coral;
  private boolean needsReverse;

    public IntakeCommand() {
        coral = Coral.getInstance();
    }

    @Override
    public void initialize() {
      System.out.println("I am in the command");

    }

    @Override
    public void execute() {
        if (coral.bb1Tripped && coral.bb2Tripped) {
            needsReverse = false;
        // Both beam breaks are tripped: full speed
            coral.motor.set(.5);
        } else if (!coral.bb1Tripped && coral.bb2Tripped) {
            needsReverse = false;
            // bb1 is not tripped, bb2 is tripped: 0.75 speed
            coral.motor.set(0.25);
        } else if (!coral.bb1Tripped && !coral.bb2Tripped && !needsReverse) {
            needsReverse = false;
            // Both beam breaks are not tripped: 0.5 speed
            coral.motor.set(0.1);
        } else if (coral.bb1Tripped && !coral.bb2Tripped) {
            // bb1 is tripped, bb2 is not tripped: stop the motor
            coral.motor.set(0);
            needsReverse = true;
        }
        else if (needsReverse && coral.bb1Tripped && !coral.bb2Tripped){
            coral.motor.set(-0.1);
        }
        else if (needsReverse && !coral.bb1Tripped && !coral.bb2Tripped){
            coral.motor.set(0.0);
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends
        coral.motor.set(0);
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        return false;
    }
}
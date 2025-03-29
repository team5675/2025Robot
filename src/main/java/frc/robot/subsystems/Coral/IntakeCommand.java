package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LED.LEDStateManager;
import frc.robot.subsystems.LED.RGB;
import frc.robot.subsystems.LED.SetLEDAnimationCommand;
import frc.robot.subsystems.LED.CustomAnimations.Pulse;
import frc.robot.subsystems.LED.CustomAnimations.ShootingLines;
import frc.robot.subsystems.LED.CustomAnimations.SolidColor;

public class IntakeCommand extends Command {

    private Coral coral;
    private boolean needsReverse;
    public Timer timer;

    public IntakeCommand() {
        coral = Coral.getInstance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        // System.out.println("I am in the command");
        Coral.intaking = true;
        timer.reset();
        // Set the led pattern to shooting lines
        // new SetLEDAnimationCommand(
        //     new ShootingLines(
        //         new RGB(Color.kHotPink),
        //         false,
        //         20,
        //         0,
        //         0,
        //         2,
        //         true
        //     )
        // ).schedule();
    }

    @Override
    public void execute() {
        if (!coral.bb1Tripped && !coral.bb2Tripped) {
            needsReverse = false;
            // Both beam breaks are tripped: full speed
            coral.motor.set(-1);
        } else if (coral.bb1Tripped && !coral.bb2Tripped) {
            needsReverse = false;
            // bb1 is not tripped, bb2 is tripped: 0.75 speed
            coral.motor.set(-0.75);
        } else if (coral.bb1Tripped && coral.bb2Tripped && !needsReverse) {
            needsReverse = false;
            // Both beam breaks are not tripped: 0.5 speed
            coral.motor.set(-0.5);
        } else if (!coral.bb1Tripped && coral.bb2Tripped && !needsReverse) {
            // bb1 is tripped, bb2 is not tripped: stop the motor
            coral.motor.set(0);
            needsReverse = true;
        } else if (needsReverse && !coral.bb1Tripped && coral.bb2Tripped) {
            coral.motor.set(0.10);
        } else if (needsReverse && coral.bb1Tripped && coral.bb2Tripped) {
            coral.motor.set(0.0);
        }

        if(needsReverse){
            timer.start();
            if(timer.hasElapsed(0.3)){
                Coral.intaking = false;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends
        coral.motor.set(0);

        new SetLEDAnimationCommand(
            new Pulse(
                new RGB(Color.kHotPink),
                0.0,
                1.0,
                0.4
            )
        ).schedule();

        Commands.waitSeconds(1).andThen(Commands.runOnce(() -> LEDStateManager.getInstance().setDefault())).schedule();
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        if (coral.bb1Tripped && coral.bb2Tripped) {
            if (needsReverse) {
                return true;
            } else return false;
        } else {
            return false;
        }
    }
}
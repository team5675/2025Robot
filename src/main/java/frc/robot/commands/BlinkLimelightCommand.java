package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class BlinkLimelightCommand extends Command {

    Timer timer;
    NetworkTableEntry led;

   public BlinkLimelightCommand() {
    timer = new Timer();
    led = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.upperLimelightName).getEntry("ledMode");
   }

   @Override
   public void initialize() {
      timer.reset();
      timer.start();
      led.setNumber(2);
   }

   @Override
   public boolean isFinished() {
      return timer.hasElapsed(0.8);
   }

   @Override
   public void end(boolean interrupted) {
      led.setNumber(0);
      timer.stop();
   }
}
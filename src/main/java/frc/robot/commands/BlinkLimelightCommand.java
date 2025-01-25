package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class BlinkLimelightCommand extends Command {
  Timer time;
  NetworkTableEntry led;
  CommandXboxController driverController;

  public BlinkLimelightCommand(CommandXboxController controller) {
    this.driverController = controller;
    this.led = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.limelightName).getEntry("ledMode");
  }

  @Override
  public void initialize() {
    time.reset();
    led.setNumber(2);
    this.driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
  }

  @Override
  public void execute() {
    time.start();
  }

  @Override
  public boolean isFinished() {
    return time.get() >= 2.0;
  }

  @Override
  public void end(boolean interrupted) {
    led.setNumber(0);
    this.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class RumbleCommand extends Command {
  CommandXboxController driverController;  
  private Timer timer = new Timer();
  private final double duration;

  public RumbleCommand() {
    driverController = RobotContainer.getDriverController();
    duration = 1.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
        timer.reset();
        timer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}

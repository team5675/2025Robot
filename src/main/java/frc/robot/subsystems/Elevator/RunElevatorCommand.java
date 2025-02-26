// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevatorCommand extends Command {
  Elevator elevator;
  double height;

  public RunElevatorCommand(double height) {
    elevator = Elevator.getInstance();
    this.height = height;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Setting target", height);
    elevator.setTarget(height);
    SmartDashboard.putNumber("Command ticks", elevator.ticksEncoder.getPosition());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

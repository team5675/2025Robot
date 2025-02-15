// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevatorCommand extends Command {
  Elevator elevator;
  double height;

  public RunElevatorCommand(double height) {
    elevator = Elevator.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevator.ticksEncoder.getPosition() > height) {
      elevator.motor.set(-0.3);
    } else {
      elevator.motor.set(0.3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.ticksEncoder.getPosition() > height) {
      elevator.motor.set(-0.3);
    } else {
      elevator.motor.set(0.3);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setTarget(height);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

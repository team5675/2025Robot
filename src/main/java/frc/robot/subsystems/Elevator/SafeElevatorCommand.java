// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SafeElevatorCommand extends Command {
  Elevator elevator;
  ElevatorLevel level;

  public SafeElevatorCommand(ElevatorLevel height) {
    elevator = Elevator.getInstance();
    this.level = height;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    elevator.setTarget(level);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return Math.abs(level.getLevel() - Elevator.getInstance().ticksEncoder.getPosition()) < 1;
  }
}

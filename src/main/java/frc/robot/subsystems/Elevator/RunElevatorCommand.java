// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.Coral;

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
    if (DriverStation.isAutonomousEnabled() && !elevator.bottomTrigger.getAsBoolean()) {
      if (Coral.getInstance().bb2Tripped && !Coral.getInstance().bb1Tripped) {
        elevator.setTarget(height);
      }
      return;
    }

    elevator.setTarget(height);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return Math.abs(height - Elevator.getInstance().ticksEncoder.getPosition()) < 1;
  }
}

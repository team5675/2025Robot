// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoIntakeCommand extends Command {
  Coral coralSubsystem;
  private IntakeCommand intakeCommand;

  public AutoIntakeCommand() {
    coralSubsystem = Coral.getInstance();
    intakeCommand = new IntakeCommand();
    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    intakeCommand.schedule(); // Start IntakeCommand
  }

  @Override
  public boolean isFinished() {
    return coralSubsystem.bb1Tripped; // End when bb1 is tripped
  }

  @Override
  public void end(boolean interrupted) {
    //intakeCommand.cancel(); // Stop intake when command ends
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj2.command.Command;

public class InstantIntake extends Command {
  Coral coralSubsystem;

  public InstantIntake() {
    coralSubsystem = Coral.getInstance();
  }

  @Override
  public void initialize() {
    new IntakeCommand();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return coralSubsystem.bb1Tripped;
  }
}

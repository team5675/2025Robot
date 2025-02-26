// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae.Algae;
import frc.robot.subsystems.Algae.AlgaeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeInCommand extends Command {

  private Algae algae;

  /** Creates a new AlgaeInCommand. */
  public AlgaeInCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    algae = Algae.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(algae.axisTicks.getPosition() < AlgaeConstants.AxisOutTicks){
      algae.AxisOut();
    }
    algae.flywheelSpin(-0.5);
    if(algae.axisTicks.getPosition() > AlgaeConstants.AxisOutTicks-2){
      algae.AxisIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

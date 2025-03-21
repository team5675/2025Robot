// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LEDStateManager;
import frc.robot.subsystems.LED.RGB;
import frc.robot.subsystems.LED.SetLEDAnimationCommand;
import frc.robot.subsystems.LED.CustomAnimations.Pulse;
import frc.robot.subsystems.LED.CustomAnimations.ShootingLines;
import frc.robot.subsystems.LED.CustomAnimations.SolidColor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeOutCommand extends Command {
  
  private Algae algae;

  public AlgaeOutCommand() {
    algae = Algae.getInstance();

    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algae.setIntake(true);

    new SetLEDAnimationCommand(
      new Pulse(
        new RGB(Color.kSeaGreen), 
        0.5, 
        1, 
        0.2
      )
    ).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Algae out");
    algae.setFlywheelSpeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.setIntake(false);
    algae.setFlywheelSpeed(0);

    LEDStateManager.getInstance().setDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

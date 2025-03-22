// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.RGB;
import frc.robot.subsystems.LED.SetLEDAnimationCommand;
import frc.robot.subsystems.LED.CustomAnimations.Pulse;
import frc.robot.subsystems.LED.CustomAnimations.SolidColor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeInCommand extends Command {

  private Algae algae;

  /** Creates a new AlgaeInCommand. */
  public AlgaeInCommand() {
    algae = Algae.getInstance();

    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    // System.out.println("Algae in");
    algae.setIntake(true);
    if (-algae.axisTicks.getPosition() < AlgaeConstants.AxisOutTicks) {
      algae.setAxisPosition(AlgaeConstants.AxisOutTicks);
      algae.setFlywheelSpeed(0.5);
    }

    // if (algae.axisTicks.getPosition() > AlgaeConstants.AxisOutTicks - 2) {
    // algae.setAxisPosition(AlgaeConstants.AxisInTicks);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.setFlywheelSpeed(0);
    algae.setIntake(false);

    new SetLEDAnimationCommand(
      new SolidColor(
          new RGB(Color.kSeaGreen)
      )
    ).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

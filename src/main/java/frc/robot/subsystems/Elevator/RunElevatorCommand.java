package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Coral.Coral;

public class RunElevatorCommand extends Command {
  Elevator elevator;
  ElevatorLevel level;

  public RunElevatorCommand(ElevatorLevel height) {
    elevator = Elevator.getInstance();
    this.level = height;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    if (DriverStation.isAutonomousEnabled() && Elevator.getInstance().ticksEncoder.getPosition() < 1) {
        Command waitThenMove = Commands.waitUntil(Coral::clearToMove)
            .andThen(() -> elevator.setTarget(level));

        waitThenMove.schedule(); // Properly schedules the command
    } else {
      elevator.setTarget(level);
    } 
  }
  

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return Math.abs(level.getLevel() - Elevator.getInstance().ticksEncoder.getPosition()) < 1;
  }
}

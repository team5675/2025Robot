package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

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

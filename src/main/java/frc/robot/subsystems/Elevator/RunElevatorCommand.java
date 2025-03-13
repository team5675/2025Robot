package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LEDStateManager;
import frc.robot.subsystems.LED.LEDStateManager.ElevatorState;

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
    var position = Elevator.getInstance().ticksEncoder.getPosition();

    if (position > level.getLevel()) {
      LEDStateManager.getInstance().setElevatorState(ElevatorState.MOVING_DOWN);
    } else if (position < level.getLevel()) {
      LEDStateManager.getInstance().setElevatorState(ElevatorState.MOVING_UP);
    }
  }

  @Override
  public void execute() {
    elevator.setTarget(level);
  }

  @Override
  public void end(boolean interrupted) {
    if (level.getName() == "RESET_HEIGHT") { 
      LEDStateManager.getInstance().setElevatorState(ElevatorState.NONE);
    } else {
      LEDStateManager.getInstance().setElevatorState(ElevatorState.AT_TARGET);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(level.getLevel() - Elevator.getInstance().ticksEncoder.getPosition()) < 1;
  }
}

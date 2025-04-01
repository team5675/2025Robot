package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.LED.LEDStateManager;
import frc.robot.subsystems.LED.LEDStateManager.LEDState;

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
      // var position = Elevator.getInstance().ticksEncoder.getPosition();

      // if (position > level.getLevel()) {
      //   LEDStateManager.getInstance().setElevatorState(ElevatorState.MOVING_DOWN);
      // } else if (position < level.getLevel()) {
      //   LEDStateManager.getInstance().setElevatorState(ElevatorState.MOVING_UP);
      // }


      elevator.setTarget(level);
    } 
  }
  

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (level.getName() != "RESET_HEIGHT") { 
      // Set at target leds
    } else {
      if (DriverStation.isAutonomous()) return; // temp 

      LEDStateManager.getInstance().setLedStateWithTimeout(LEDState.ELEVATOR_RESET, 1);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(level.getLevel() - Elevator.getInstance().ticksEncoder.getPosition()) < 1;
  }
}

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED.CustomAnimations.CustomizableRainbow;
import frc.robot.subsystems.LED.CustomAnimations.Pulse;
import frc.robot.subsystems.LED.CustomAnimations.RainbowShootingLines;
import frc.robot.subsystems.LED.CustomAnimations.ShootingLines;
import frc.robot.subsystems.LED.CustomAnimations.SolidColor;

public class LEDStateManager extends SubsystemBase {
  private final LED ledSubsystem;

  // Timers to track how long we've been in certain states
  private double linedUpStartTime = 0;
  private double atTargetStartTime = 0;

  // Flag to track if we need to update the LED pattern
  private boolean needsUpdate = true;

  // Current state for each system
  private LineupState lineupState = LineupState.NONE;
  private ElevatorState elevatorState = ElevatorState.NONE;

  // Time in seconds before reverting to NONE state
  private static final double STATE_TIMEOUT = 0.5;

  public LEDStateManager() {
    ledSubsystem = LED.getInstance();
    updateLEDPattern();
  }

  public enum LineupState {
    NONE,
    LINING_UP,
    LINED_UP
  }

  public void setLineupState(LineupState state) {
    // Only update if the state is actually changing
    if (state != lineupState) {
      lineupState = state;

      // If we're entering LINED_UP state, record the start time
      if (state == LineupState.LINED_UP) {
        linedUpStartTime = Timer.getFPGATimestamp();
      }

      // Flag that we need to update the pattern
      needsUpdate = true;
    }
  }

  public enum ElevatorState {
    NONE,
    MOVING_UP,
    MOVING_DOWN,
    AT_TARGET
  }

  public void setElevatorState(ElevatorState state) {
    if (state != elevatorState) {
      elevatorState = state;

      if (state == ElevatorState.AT_TARGET) {
        atTargetStartTime = Timer.getFPGATimestamp();
      }

      needsUpdate = true;
    }
  }

  @Override
  public void periodic() {
    if (!RobotContainer.getLEDsEnabled()) {
      return;
    }

    double currentTime = Timer.getFPGATimestamp();
    boolean stateChanged = false;

    // Check if we need to automatically revert from LINED_UP to NONE
    if (lineupState == LineupState.LINED_UP) {
      if (currentTime - linedUpStartTime >= STATE_TIMEOUT) {
        lineupState = LineupState.NONE;
        stateChanged = true;
      }
    }

    // Check if we need to automatically revert from AT_TARGET to NONE
    if (elevatorState == ElevatorState.AT_TARGET) {
      if (currentTime - atTargetStartTime >= STATE_TIMEOUT) {
        elevatorState = ElevatorState.NONE;
        stateChanged = true;
      }
    }

    if (stateChanged || needsUpdate) {
      updateLEDPattern();
      needsUpdate = false;
    }
  }

  /**
   * Updates the LED pattern based on the current lineup and elevator states.
   * Elevator state determines the pattern, while lineup state determines the color.
   */
  private void updateLEDPattern() {
    System.out.println("Updating LED pattern");
    switch (elevatorState) {
      case NONE:
        // Use solid rainbow pattern
        if (lineupState == LineupState.NONE) {
          // COMMENT: Set LEDs to rainbow pattern
          new SetLEDAnimationCommand(
            new CustomizableRainbow(
              CustomizableRainbow.RainbowType.PASTEL,
              CustomizableRainbow.PatternType.CONTINUOUS,
              CustomizableRainbow.Direction.FORWARD,
               0.3,
              1.0
            )
          );
        } else if (lineupState == LineupState.LINING_UP) {
          // COMMENT: Set LEDs to solid red pattern
          new SetLEDAnimationCommand(
            new SolidColor(
              new RGB(Color.kRed)
            )
          );
        } else if (lineupState == LineupState.LINED_UP) {
          // COMMENT: Set LEDs to solid blue pattern
          new SetLEDAnimationCommand(
              new Pulse(
                new RGB(Color.kAliceBlue),
                  0.5, 
                  1.0, 
                  0.4
                )
            );
        }
        break;
        
      case MOVING_UP:
        // Use shooting lines effect going forward
        if (lineupState == LineupState.NONE) {
          // COMMENT: Set LEDs to rainbow shooting lines pattern in FORWARD direction
          new SetLEDAnimationCommand(
              new RainbowShootingLines(
                RainbowShootingLines.RainbowType.PASTEL_RAINBOW,
                RainbowShootingLines.ColorDistribution.PER_LINE,
                 15,
                 0.0,
                 0.0,
                 0.5,
                 0.0,
                 true,
                 true
              )
            );
        } else if (lineupState == LineupState.LINING_UP) {
          // COMMENT: Set LEDs to red shooting lines pattern in FORWARD direction
           new SetLEDAnimationCommand(
              new ShootingLines(
                new RGB(Color.kRed),
                true,
                15,
                0.0,
                0,
                0.5,
                true
              ));
        } else if (lineupState == LineupState.LINED_UP) {
          new SetLEDAnimationCommand(
              new ShootingLines(
                  new RGB(Color.kAliceBlue),
                  true,
                  15,
                  0.0,
                  0,
                  0.5,
                  true));
        }
        break;
        
      case MOVING_DOWN:
        // Use shooting lines effect going backward
        if (lineupState == LineupState.NONE) {
          // COMMENT: Set LEDs to rainbow shooting lines pattern in BACKWARD direction
          // ledSubsystem.setAnimation(new RainbowShootingLinesAnimation(false)); // false for backward
        } else if (lineupState == LineupState.LINING_UP) {
          // COMMENT: Set LEDs to red shooting lines pattern in BACKWARD direction
          new SetLEDAnimationCommand(
              new ShootingLines(
                  new RGB(Color.kRed),
                  false,
                  15,
                  0.0,
                  0,
                  0.5,
                  true));
        } else if (lineupState == LineupState.LINED_UP) {
          // COMMENT: Set LEDs to blue shooting lines pattern in BACKWARD direction
          new SetLEDAnimationCommand(
              new ShootingLines(
                  new RGB(Color.kAliceBlue),
                  false,
                  15,
                  0.0,
                  0,
                  0.5,
                  true));  
        }
        break;
        
      case AT_TARGET:
        // Use pulsing effect
        if (lineupState == LineupState.NONE) {
          // COMMENT: Set LEDs to rainbow pulsing pattern
          
          new SetLEDAnimationCommand(
            new CustomizableRainbow(
              CustomizableRainbow.RainbowType.PASTEL,
              CustomizableRainbow.PatternType.CONTINUOUS,
              CustomizableRainbow.Direction.FORWARD,
              0.3,
              1.0
              )
            );

        } else if (lineupState == LineupState.LINING_UP) {
          // COMMENT: Set LEDs to red pulsing pattern
          new SetLEDAnimationCommand(
              new SolidColor(
                  new RGB(Color.kRed)));
              
        } else if (lineupState == LineupState.LINED_UP) {
          // COMMENT: Set LEDs to blue pulsing pattern
          new SetLEDAnimationCommand(
              new SolidColor(
                  new RGB(Color.kAliceBlue)));
        }
        break;
    }
  }

  public LineupState getLineupState() {
    return lineupState;
  }

  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  private static LEDStateManager instance;

  public static LEDStateManager getInstance() {
    if (instance == null) {
      instance = new LEDStateManager();
    }
    return instance;
  }
}
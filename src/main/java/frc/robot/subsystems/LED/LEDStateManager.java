package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LED.CustomAnimations.Blink;
import frc.robot.subsystems.LED.CustomAnimations.CustomizableRainbow;
import frc.robot.subsystems.LED.CustomAnimations.Pulse;
import frc.robot.subsystems.LED.CustomAnimations.RainbowPulse;
import frc.robot.subsystems.LED.CustomAnimations.RainbowShootingLines;
import frc.robot.subsystems.LED.CustomAnimations.SolidColor;

public class LEDStateManager extends SubsystemBase {
  private final CustomizableRainbow BASIC_RAINBOW;
  private final SolidColor SOLID_RED;
  private final SolidColor SOLID_BLUE;
  private final SolidColor SOLID_ALICEBLUE;
  private final SolidColor SOLID_PURPLE;
  private final SolidColor SOLID_SEAGREEN;
  private final Pulse PULSE_ALICEBLUE;
  private final Pulse PULSE_PURPLE;
  public final Pulse PULSE_GREEN_LINEUPDONE;
  private final Pulse PULSE_RED_SYSTEM_HEALTH_BAD;
  private final RainbowShootingLines RAINBOW_SHOOTING_LINES_FORWARD;
  private final Pulse PULSE_ALGAE_IN;
  private final RainbowPulse PULSE_CLIMBING;
  private final Pulse PULSE_INTAKED;

  public final RainbowShootingLines STARTING_SHOOTING_LINES;

  public final Blink BLINK_RESET;

  private final CustomizableRainbow DEFAULT;

  private final LED ledSubsystem;

  // Timers to track how long we've been in certain states
  private double animationStartTime = 0;
  private double atTargetStartTime = 0;


  private double resetTimeout = 1;

  public LEDStateManager() {
    ledSubsystem = LED.getInstance();
    updateLEDPatternPeriodic();

    BASIC_RAINBOW = new CustomizableRainbow(
      CustomizableRainbow.RainbowType.COOL,
      CustomizableRainbow.PatternType.CONTINUOUS,
      CustomizableRainbow.Direction.FORWARD,
      0.4,
      1.0
    );

    SOLID_RED = new SolidColor(new RGB(Color.kRed));
    SOLID_BLUE = new SolidColor(new RGB(Color.kBlue));
    SOLID_ALICEBLUE = new SolidColor(new RGB(Color.kAliceBlue));
    SOLID_PURPLE = new SolidColor(new RGB(Color.kPurple));
    SOLID_SEAGREEN = new SolidColor(new RGB(Color.kSeaGreen));

    PULSE_ALICEBLUE = new Pulse(
      new RGB(Color.kAliceBlue),
      0.5,
      1.0,
      0.4
    );

    PULSE_PURPLE = new Pulse(
      new RGB(Color.kAliceBlue),
      0.5,
      1.0,
      0.3
    );

    PULSE_GREEN_LINEUPDONE = new Pulse(
      new RGB(Color.kGreen),
      0.0,
      1.0,
      0.4
    );

    PULSE_RED_SYSTEM_HEALTH_BAD = new Pulse(
      new RGB(Color.kRed),
      0.0,
      1.0,
      2
    );

    RAINBOW_SHOOTING_LINES_FORWARD = new RainbowShootingLines(
      RainbowShootingLines.RainbowType.PASTEL_RAINBOW,
      RainbowShootingLines.ColorDistribution.PER_LINE,
      15,
      0.0,
      0.0,
      0.5,
      0.0,
      true,
      true
    );


    // SHOOTING_LINES_RED_FORWARD = new ShootingLines(
    //   new RGB(Color.kRed),
    //   true,
    //   30,
    //   0.0,
    //   5,
    //   0.5,
    //   true
    // );

    // SHOOTING_LINES_BLUE_FORWARD = new ShootingLines(
    //   new RGB(Color.kBlue),
    //   true,
    //   15,
    //   0.0,
    //   0,
    //   0.5,
    //   true
    // );

    // SHOOTING_LINES_RED_BACKWARD = new ShootingLines(
    //   new RGB(Color.kRed),
    //   false,
    //   15,
    //   0.0,
    //   0,
    //   0.5,
    //   true
    // );

    // SHOOTING_LINES_BLUE_BACKWARD = new ShootingLines(
    //   new RGB(Color.kBlue),
    //   false,
    //   15,
    //   0.0,
    //   0,
    //   0.5,
    //   true
    // );

    
    // SHOOTING_LINES_ALICEBLUE_BACKWARD = new ShootingLines(
      //   new RGB(Color.kAliceBlue),
      //   false,
      //   15,
      //   0.0,
      //   0,
      //   0.5,
      //   true
      // );
      
      // SHOOTING_LINES_ALICEBLUE_FORWARD = new ShootingLines(
        //   new RGB(Color.kAliceBlue),
        //   true,
        //   15,
        //   0.0,
        //   0,
        //   0.5,
        //   true
        // );
        
    DEFAULT = new CustomizableRainbow(
      CustomizableRainbow.RainbowType.COOL,
      CustomizableRainbow.PatternType.CONTINUOUS,
      CustomizableRainbow.Direction.FORWARD,
      0.5,
      1.0
    );

    BLINK_RESET = new Blink(new RGB(Color.kDeepSkyBlue), 0.1, 0.1);

    STARTING_SHOOTING_LINES = new RainbowShootingLines(
      RainbowShootingLines.RainbowType.PASTEL_RAINBOW, 
      RainbowShootingLines.ColorDistribution.PER_LINE, 
      RainbowShootingLines.DirectionType.RANDOM,
      30, 
      0, 
      0.5, 
      3, 
      0, 
      false
    );

    PULSE_ALGAE_IN = new Pulse(
      new RGB(Color.kSeaGreen), 
      0.5, 
      1, 
      0.2
    );

    PULSE_CLIMBING = new RainbowPulse(
      RainbowPulse.RainbowType.PASTEL_RAINBOW, 
      0, 
      1, 
      1.3,
      0
    );

    PULSE_INTAKED = new Pulse(
      new RGB(Color.kHotPink),
      0.0,
      1.0,
      0.4
    );
  }

  public enum LEDState {
    NONE,
    AUTO_AT_STATION, // auto only
    LINING_UP,
    LINED_UP,
    ALGAEING,
    ALGAED,
    CLIMBING,
    CLIMBED,
    INTAKED,
    ELEVATOR_RESET
  }

  // if we need to update the LED pattern
  private boolean needsUpdate = true;

  // Current state for each system
  private LEDState ledState = LEDState.NONE;

  public void setLedState(LEDState state) {
    if (state != ledState) {

      if (DriverStation.isAutonomous() && state != LEDState.AUTO_AT_STATION) return;

      resetTimeout = 0;
      ledState = state;

      // Flag that we need to update the pattern
      needsUpdate = true;
    }
  }

  public void setLedStateWithTimeout(LEDState state, double timeout) {
    if (state != ledState) {

      if (DriverStation.isAutonomous() && state != LEDState.AUTO_AT_STATION) return;

      ledState = state;
      animationStartTime = Timer.getFPGATimestamp();
      resetTimeout = timeout;

      // Flag that we need to update the pattern
      needsUpdate = true;
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isAutonomous()) return;

    double currentTime = Timer.getFPGATimestamp();

    // Check if we need to automatically revert from LINED_UP to NONE
  
    if (resetTimeout != 0 && currentTime - animationStartTime >= resetTimeout) {
      ledState = LEDState.NONE;
      needsUpdate = true;
      resetTimeout = 0;
    }

    if (needsUpdate) {
      try {
        updateLEDPatternPeriodic();
      } catch (Exception e) {
        //System.out.println("Error Setting LED Pattern!");
      }
      needsUpdate = false;
    }
  }

  private void updateLEDPatternPeriodic() {
      switch (ledState) {
        case NONE:
          new SetLEDAnimationCommand(DEFAULT).schedule();
          break;
        case AUTO_AT_STATION:
          new SetLEDAnimationCommand(PULSE_GREEN_LINEUPDONE).schedule();
          break;
        case LINING_UP:
          new SetLEDAnimationCommand(SOLID_PURPLE).schedule();
          break;
        case LINED_UP:
          new SetLEDAnimationCommand(PULSE_GREEN_LINEUPDONE).schedule();
          break;
        case ALGAEING:
          new SetLEDAnimationCommand(PULSE_ALGAE_IN).schedule();
          break;
        case ALGAED:
          new SetLEDAnimationCommand(SOLID_SEAGREEN).schedule();
          break;
        case CLIMBED:
          new SetLEDAnimationCommand(STARTING_SHOOTING_LINES).schedule();
          break;
        case CLIMBING:
          new SetLEDAnimationCommand(PULSE_CLIMBING).schedule();
          break;
        case INTAKED:
          new SetLEDAnimationCommand(PULSE_INTAKED).schedule();
          break;
        case ELEVATOR_RESET:
          new SetLEDAnimationCommand(BLINK_RESET).schedule();
          break;
        default:
          break;
      }
  }

  public LEDState getLedState() {
    return this.ledState;
  }

  private static LEDStateManager instance;

  public static LEDStateManager getInstance() {
    if (instance == null) {
      instance = new LEDStateManager();
    }
    return instance;
  }
}
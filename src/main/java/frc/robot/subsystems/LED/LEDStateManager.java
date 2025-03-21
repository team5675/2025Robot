package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.LED.CustomAnimations.Blink;
import frc.robot.subsystems.LED.CustomAnimations.CustomizableRainbow;
import frc.robot.subsystems.LED.CustomAnimations.Pulse;
import frc.robot.subsystems.LED.CustomAnimations.RainbowGlitch;
import frc.robot.subsystems.LED.CustomAnimations.RainbowShootingLines;
import frc.robot.subsystems.LED.CustomAnimations.ShootingLines;
import frc.robot.subsystems.LED.CustomAnimations.SolidColor;
import frc.robot.subsystems.LED.CustomAnimations.RainbowGlitch.ColorDistribution;

public class LEDStateManager extends SubsystemBase {
  private final CustomizableRainbow BASIC_RAINBOW;
  private final SolidColor SOLID_RED;
  private final SolidColor SOLID_BLUE;
  private final SolidColor SOLID_ALICEBLUE;
  private final SolidColor SOLID_PURPLE;
  private final Pulse PULSE_ALICEBLUE;
  private final Pulse PULSE_PURPLE;
  private final Pulse PULSE_GREEN_LINEUPDONE;
  private final Pulse PULSE_RED_SYSTEM_HEALTH_BAD;
  private final RainbowShootingLines RAINBOW_SHOOTING_LINES_FORWARD;

  public final RainbowShootingLines STARTING_SHOOTING_LINES;

  public final Blink BLINK_RESET;

  private final CustomizableRainbow BASIC_PASTEL;

  private final LED ledSubsystem;

  // Timers to track how long we've been in certain states
  private double linedUpStartTime = 0;
  private double atTargetStartTime = 0;


  // Time in seconds before reverting to NONE state
  private static final double STATE_TIMEOUT_LINEUP = 1;

  public LEDStateManager() {
    ledSubsystem = LED.getInstance();
    updateLEDPatternPeriodic();

    BASIC_RAINBOW = new CustomizableRainbow(
      CustomizableRainbow.RainbowType.PASTEL,
      CustomizableRainbow.PatternType.CONTINUOUS,
      CustomizableRainbow.Direction.FORWARD,
      0.3,
      1.0
    );

    SOLID_RED = new SolidColor(new RGB(Color.kRed));
    SOLID_BLUE = new SolidColor(new RGB(Color.kBlue));
    SOLID_ALICEBLUE = new SolidColor(new RGB(Color.kAliceBlue));
    SOLID_PURPLE = new SolidColor(new RGB(Color.kPurple));

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
        
    BASIC_PASTEL = new CustomizableRainbow(
      CustomizableRainbow.RainbowType.NEON,
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
      2, 
      3, 
      0, 
      false
    );
  }

  public enum LineupState {
    NONE,
    LINING_UP,
    LINED_UP
  }

  // Flag to track if we need to update the LED pattern
  private boolean needsUpdate = true;

  // Current state for each system
  private LineupState lineupState = LineupState.NONE;
  private SystemState systemState = SystemState.GOOD;

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

  public enum SystemState {
    GOOD,
    BAD
  }
  

  public void setSystemState(SystemState state) {
    if (state != systemState) {
      systemState = state;

      needsUpdate = true;
    }
  }


  @Override
  public void periodic() {
    // if (!RobotContainer.getLEDsEnabled()) {
    //   return;
    // }
    // if (DriverStation.isDisabled()) {
    //   setDefault();
    // }

    
    // if (!DriverStation.isEnabled()) {
    //   System.err.println();
    //   // SYSTEM CHECK !!
    //   // is bb1 tripped but not bb2
    //   setSystemState(SystemState.GOOD);

    //   // if a coral is not properly seated
    //   if (!(Coral.getInstance().bb2Tripped && !Coral.getInstance().bb1Tripped)) {
    //     setSystemState(SystemState.BAD);
    //   }

    //   if (needsUpdate || LED.getInstance().currentAnimation == null) {
    //     try {
    //       updateSystemLEDPattern();
    //     } catch (Exception e) {
    //       //System.out.println("Error Setting LED Pattern!");
    //     }
    //   }

    //   return;
    // }

    if (DriverStation.isAutonomous()) return;

    // if (!DriverStation.isTeleopEnabled()) {
    //   return;
    // }

    double currentTime = Timer.getFPGATimestamp();
    boolean stateChanged = false;

    // Check if we need to automatically revert from LINED_UP to NONE
    if (lineupState == LineupState.LINED_UP) {
      if (currentTime - linedUpStartTime >= STATE_TIMEOUT_LINEUP) {
        lineupState = LineupState.NONE;
        stateChanged = true;
      }
    }

    if (stateChanged || needsUpdate) {
      try {
        updateLEDPatternPeriodic();
      } catch (Exception e) {
        //System.out.println("Error Setting LED Pattern!");
      }
      needsUpdate = false;
    }
  }

  /**
   * Updates the LED pattern based on the current lineup and elevator states.
   * Elevator state determines the pattern, while lineup state determines the color.
   */
  private void updateLEDPatternPeriodic() {
    //System.out.println("Updating LED pattern");
      if (lineupState == LineupState.NONE) {
          // COMMENT: Set LEDs to rainbow pattern
        setDefault();

      } else if (lineupState == LineupState.LINING_UP) {
          // COMMENT: Set LEDs to solid red pattern
        new SetLEDAnimationCommand(SOLID_PURPLE).schedule();

      } else if (lineupState == LineupState.LINED_UP) {
          // COMMENT: Set LEDs to pulsing blue
        new SetLEDAnimationCommand(PULSE_GREEN_LINEUPDONE).schedule();
      }
  }

  // private void updateSystemLEDPattern() {
  //   switch (systemState) {
  //     case GOOD:
  //       new SetLEDAnimationCommand(STARTING_SHOOTING_LINES).schedule();
  //       break;
  //     case BAD:
  //       new SetLEDAnimationCommand(PULSE_RED_SYSTEM_HEALTH_BAD).schedule();
  //       break;
  //   }
  // }

  public void setDefault() {
    new SetLEDAnimationCommand(BASIC_PASTEL).schedule();
  }

  public void setDefaultDisabled() {
    new SetLEDAnimationCommand(STARTING_SHOOTING_LINES).schedule();
  }

  public LineupState getLineupState() {
    return lineupState;
  }

  private static LEDStateManager instance;

  public static LEDStateManager getInstance() {
    if (instance == null) {
      instance = new LEDStateManager();
    }
    return instance;
  }
}
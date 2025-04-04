package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Algae.AlgaeInCommand;
import frc.robot.subsystems.Algae.AlgaeOutCommand;
import frc.robot.subsystems.Elevator.RunElevatorCommand;
import frc.robot.subsystems.LED.LEDStateManager;
import frc.robot.subsystems.LED.LEDStateManager.LEDState;
import frc.robot.subsystems.Climber.SetClimbCommand;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.Coral.IntakeCommand;
import frc.robot.subsystems.Coral.PlaceCommand;
import frc.robot.subsystems.Coral.AutoIntakeCommand;
import frc.robot.subsystems.Algae.Algae;
import frc.robot.subsystems.Climber.ClimbCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.CloseClawCommand;
import frc.robot.subsystems.Climber.OpenClawCommand;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.DriveToPoseCommand;
import frc.robot.subsystems.Swerve.Telemetry;
import frc.robot.subsystems.Swerve.TunerConstants;
import frc.robot.commands.BlinkLimelightCommand;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private static CommandXboxController driverController;
    public static CommandXboxController getDriverController() {
        if (driverController == null) { 
            driverController = new CommandXboxController(0); 
        }
        return driverController;
    }

    private final Joystick IpacSide1 = new Joystick(1);
    private final Joystick IpacSide2 = new Joystick(2);

    JoystickButton ElevatorReset = new JoystickButton(IpacSide1, 1);
    JoystickButton level1 = new JoystickButton(IpacSide1, 2);
    JoystickButton level2 = new JoystickButton(IpacSide1, 3);
    JoystickButton level3 = new JoystickButton(IpacSide1, 4);
    JoystickButton level4 = new JoystickButton(IpacSide1, 5);
    JoystickButton AlgaeLow = new JoystickButton(IpacSide1, 6);
    JoystickButton AlgaeHigh = new JoystickButton(IpacSide1, 7);
    JoystickButton CoralIn = new JoystickButton(IpacSide2, 1);
    JoystickButton Score = new JoystickButton(IpacSide2,2);
    JoystickButton CoralReset = new JoystickButton(IpacSide2, 3);
    JoystickButton AlgaeIn = new JoystickButton(IpacSide2, 4);
    JoystickButton AlgaeHold = new JoystickButton(IpacSide2, 5);
    JoystickButton AlgaeOut = new JoystickButton(IpacSide2, 6);
    JoystickButton CloseClaw = new JoystickButton(IpacSide2, 7);
    JoystickButton Climb = new JoystickButton(IpacSide2, 8);
    JoystickButton SetClimber = new JoystickButton(IpacSide2, 9);
    JoystickButton OpenClaw = new JoystickButton(IpacSide2, 10);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
 /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("IntakeCommand", new AutoIntakeCommand());
        NamedCommands.registerCommand("PlaceCommand", new PlaceCommand());
        NamedCommands.registerCommand("ElevatorL1", new RunElevatorCommand(ElevatorLevel.L1_HEIGHT));
        NamedCommands.registerCommand("ElevatorL2", new RunElevatorCommand(ElevatorLevel.L2_HEIGHT));
        NamedCommands.registerCommand("ElevatorL3", new RunElevatorCommand(ElevatorLevel.L3_HEIGHT));
        NamedCommands.registerCommand("ElevatorL4", new RunElevatorCommand(ElevatorLevel.L4_HEIGHT));
        NamedCommands.registerCommand("ElevatorReset", new RunElevatorCommand(ElevatorLevel.RESET_HEIGHT));
        NamedCommands.registerCommand("AlgaeLow", new RunElevatorCommand(ElevatorLevel.ALGAE_LOW_HEIGHT));
        NamedCommands.registerCommand("AlgaeHigh", new RunElevatorCommand(ElevatorLevel.ALGAE_HIGH_HEIGHT));
        NamedCommands.registerCommand("AlgaeIn", new AlgaeInCommand());
        NamedCommands.registerCommand("AlgaeOut", new AlgaeOutCommand());
        NamedCommands.registerCommand("AlgaeHold", Commands.runOnce(() -> Algae.getInstance().setFlywheelSpeed(0.1)));
        NamedCommands.registerCommand("BlinkLimelightCommand", new BlinkLimelightCommand());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
         if(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {

             drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(getDriverController().getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(getDriverController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-getDriverController().getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); } else {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-getDriverController().getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-getDriverController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-getDriverController().getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); }
       
        // Reset the field-centric heading on left bumper press and resets gyro on b button press
       getDriverController().b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Auto Lineups
        getDriverController().leftTrigger().and(getDriverController().rightTrigger().negate())
            .whileTrue(new DriveToPoseCommand(drivetrain, "Left", () -> true, false));

        getDriverController().rightTrigger().and(getDriverController().leftTrigger().negate())
            .whileTrue(new DriveToPoseCommand(drivetrain, "Right", () -> true, false));

        getDriverController().rightTrigger().and(getDriverController().leftTrigger())
        .whileTrue(new DriveToPoseCommand(drivetrain, "Algae", () -> true, true));

        getDriverController().leftBumper().and(getDriverController().rightBumper().negate())
            .whileTrue(new DriveToPoseCommand(drivetrain, "LeftCoralStation", () -> false, false));

        getDriverController().rightBumper().and(getDriverController().leftBumper().negate())
            .whileTrue(new DriveToPoseCommand(drivetrain, "RightCoralStation", () -> false, false));

        getDriverController().povUp().whileTrue(new DriveToPoseCommand(drivetrain, "MidBarge", () -> false, false));
        getDriverController().povLeft().whileTrue(new DriveToPoseCommand(drivetrain, "LeftBarge",() -> false, false));
        getDriverController().povRight().whileTrue(new DriveToPoseCommand(drivetrain, "RightBarge",() -> false, false));

        // Aux Button Board
        CloseClaw.whileTrue(new CloseClawCommand(Climber.getInstance()));
        Climb.whileTrue(new ClimbCommand(Climber.getInstance()));
        SetClimber.whileTrue(new SetClimbCommand(Climber.getInstance()));
        OpenClaw.whileTrue(new OpenClawCommand(Climber.getInstance()));

        // Coral
        CoralIn.whileTrue(new IntakeCommand());
        Score.whileTrue(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(-1)));
        Score.whileFalse(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(0)));
        CoralReset.whileTrue(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(1)));
        CoralReset.whileFalse(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(0)));

        // Algae
        AlgaeIn.whileTrue(new AlgaeInCommand());
        AlgaeOut.whileTrue(new AlgaeOutCommand());
        AlgaeHold.whileTrue(Commands.runOnce(() -> Algae.getInstance().setFlywheelSpeed(0.1)));
        AlgaeHold.whileFalse(Commands.runOnce(() -> Algae.getInstance().setFlywheelSpeed(0)));

        AlgaeHigh.onTrue(new RunElevatorCommand(ElevatorLevel.ALGAE_HIGH_HEIGHT));
        AlgaeLow.onTrue(new RunElevatorCommand(ElevatorLevel.ALGAE_LOW_HEIGHT));

        // Elevator      
        level1.onTrue(new RunElevatorCommand(ElevatorLevel.L1_HEIGHT));
        level2.onTrue(new RunElevatorCommand(ElevatorLevel.L2_HEIGHT));
        level3.onTrue(new RunElevatorCommand(ElevatorLevel.L3_HEIGHT));
        level4.onTrue(new RunElevatorCommand(ElevatorLevel.L4_HEIGHT));
        ElevatorReset.onTrue(new RunElevatorCommand(ElevatorLevel.RESET_HEIGHT));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
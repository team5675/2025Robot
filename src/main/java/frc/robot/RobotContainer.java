// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
//import frc.robot.commands.ClimbCommand;


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

    private final CommandXboxController auxController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

 /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public Command pathfindingCommand;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("P");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand());
        //NamedCommands.registerCommand("Missed Intake", );
        NamedCommands.registerCommand("PlaceCommand", Coral.PlaceCommand());
        NamedCommands.registerCommand("ElevatorL1", Elevator.setTargetCommand(ElevatorConstants.L1_HEIGHT));
        NamedCommands.registerCommand("ElevatorL2", Elevator.setTargetCommand(ElevatorConstants.L2_HEIGHT));
        NamedCommands.registerCommand("ElevatorL3", Elevator.setTargetCommand(ElevatorConstants.L3_HEIGHT));
        NamedCommands.registerCommand("ElevatorL4", Elevator.setTargetCommand(ElevatorConstants.L4_HEIGHT));


        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-getDriverController().getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-getDriverController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-getDriverController().getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        getDriverController().a().whileTrue(drivetrain.applyRequest(() -> brake));
        getDriverController().b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-getDriverController().getLeftY(), -getDriverController().getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        getDriverController().back().and(getDriverController().y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        getDriverController().back().and(getDriverController().x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        getDriverController().start().and(getDriverController().y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        getDriverController().start().and(getDriverController().x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        getDriverController().leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // Driver
        getDriverController().leftTrigger()
            .whileTrue(new DriveToPoseCommand(drivetrain, "left"));

        getDriverController().rightTrigger()
            .whileTrue(new DriveToPoseCommand(drivetrain, "right"));

        // Aux Button Board

        // Climber
        // auxController.y().onTrue(Climber.getInstance().runOnce(() -> Climber.getInstance().runClaw()));
        // auxController.x();

        // Coral
        // auxController.rightTrigger().whileTrue(new IntakeCommand());
        // auxController.a().whileTrue(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(1)));
        // auxController.a().whileFalse(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(0)));
        // auxController.b().whileTrue(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(-1)));
        // auxController.b().whileFalse(Coral.getInstance().runOnce(() -> Coral.getInstance().motor.set(0)));

        // Algae
        // auxController.leftBumper().whileTrue(Algae.getInstance().runOnce(() -> Algae.getInstance().flywheelSpin(-0.2)));
        // auxController.leftBumper().whileFalse(Algae.getInstance().runOnce(() -> Algae.getInstance().flywheelSpin(0)));
        // auxController.rightBumper().whileTrue(Algae.getInstance().runOnce(() -> Algae.getInstance().flywheelSpin(0.2)));
        // auxController.rightBumper().whileFalse(Algae.getInstance().runOnce(() -> Algae.getInstance().flywheelSpin(0)));
        
        // Elevator      
        //auxController.povRight().onTrue(Commands.runOnce(() -> Elevator.getInstance().setTarget(ElevatorConstants.L3_HEIGHT)));
        //auxController.povLeft().onTrue(Commands.runOnce(() -> Elevator.getInstance().setTarget(ElevatorConstants.L4_HEIGHT)));
        //auxController.povUpRight().onTrue(Commands.runOnce(() -> Elevator.getInstance().setTarget(ElevatorConstants.ALGAE_HIGH)));
        //auxController.povUpLeft().onTrue(Commands.runOnce(() -> Elevator.getInstance().setTarget(ElevatorConstants.ALGAE_LOW)));
        getDriverController().leftBumper().onTrue(Commands.runOnce(() -> Elevator.getInstance().setTarget(ElevatorConstants.L1_HEIGHT)));
        getDriverController().rightBumper().onTrue(Commands.runOnce(() -> Elevator.getInstance().setTarget(ElevatorConstants.L2_HEIGHT)));

        getDriverController().povLeft().onTrue(Commands.runOnce(() -> Elevator.getInstance().motor.set(-0.1)));
        getDriverController().povLeft().onFalse(Commands.runOnce(() -> Elevator.getInstance().motor.set(0)));

        getDriverController().povRight().onTrue(Commands.runOnce(() -> Elevator.getInstance().motor.set(0.1)));
        getDriverController().povRight().onFalse(Commands.runOnce(() -> Elevator.getInstance().motor.set(0)));
      //  getDriverController().x().whileTrue(Commands.runOnce(() -> ClimbCommand.getInstance().));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
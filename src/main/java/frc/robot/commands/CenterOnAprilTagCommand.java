// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.LimelightPolling;
import frc.robot.subsystems.LimelightPolling;

import com.ctre.phoenix6.swerve.SwerveRequest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterOnAprilTagCommand extends Command {
  private double kTolerance = Constants.LimelightConstants.kTolerance;
  public final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric drive;

  public CenterOnAprilTagCommand(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive) {
    this.drivetrain = driveTrain;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightCommand limelight = LimelightPolling.getInstance().tables.get(0);

    //System.out.printf("CenterOnAprilTag: April Tag ID: %.2f", limelight.tid);
    //System.out.println();

    // System.out.println(this.aprilTagID);
    if (limelight.tid == -1) {
      System.out.println("CenterOnAprilTag: No April Tag Detected.");
      return;
    }
    
    this.tx = limelight.tx; // Don't use this; it's for the isFinished function

    if (limelight.tx > this.kTolerance) {
      System.out.println("CenterOnAprilTag: AprilTag right");
            // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        drive.withVelocityX(0) // Drive forward with negative Y (forward)
        .withVelocityY((.1 - limelight.tx * 0.1) * 0) // Drive left with negative X (left)
          .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      );
    }
    if (limelight.tx < -this.kTolerance) {
      System.out.println("CenterOnAprilTag: AprilTag left");
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
          drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY((0 - (-limelight.tx) * 0.1)) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private double tx;
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(this.tx) <= this.kTolerance) {
      return true;
    }
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.LimelightPolling;
import frc.robot.subsystems.LimelightPolling;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterOnAprilTagCommand extends Command {
  private double kTolerance = Constants.LimelightConstants.kTolerance;
  public final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric driveRequest;

  public CenterOnAprilTagCommand(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive) {
    this.drivetrain = driveTrain;
    this.driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SwerveModule.SteerRequestType.Position);
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

    SmartDashboard.putNumber("AprilTag TX", limelight.tx);
    SmartDashboard.putNumber("AprilTag TY", limelight.ty);
    SmartDashboard.putNumber("Attempted Velo:", driveRequest.VelocityY);
    System.out.println();

    this.tx = limelight.tx; // Don't use this; it's for the isFinished function

    if (limelight.tx > this.kTolerance) {
      System.out.println("CenterOnAprilTag: AprilTag left");
      double strafeVelocity = limelight.tx / 20;
      if (strafeVelocity < Constants.LimelightConstants.minStrafe && strafeVelocity >= 0) {
        strafeVelocity *= 2;
      }
      drivetrain.setControl(
         driveRequest
            .withVelocityX(0) 
            .withVelocityY(strafeVelocity)//(.1 - limelight.tx * 0.1) * 0.5 
            .withRotationalRate(0)
      );

      System.out.println(driveRequest.VelocityY);
    }
    if (limelight.tx < -this.kTolerance) {
      System.out.println("CenterOnAprilTag: AprilTag right");
      double strafeVelocity = limelight.tx / 20;
      if (strafeVelocity <= 0 && strafeVelocity > -Constants.LimelightConstants.minStrafe){
        strafeVelocity *= 2;
      }
        drivetrain.setControl(
          driveRequest
            .withVelocityX(0) 
            .withVelocityY(strafeVelocity)
            .withRotationalRate(0)
        );
        
        SmartDashboard.putNumber("Attempted Velo:", driveRequest.VelocityY);
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
      System.out.printf("CenterOnAprilTag: Lineup complete at TX %.5f", this.tx);
      System.out.println();
      return true;
    }
    return false; 
  }
}

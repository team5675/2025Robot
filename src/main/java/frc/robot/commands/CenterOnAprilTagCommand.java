// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

import java.util.function.Function;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterOnAprilTagCommand extends Command {
  private double kTolerance = Constants.LimelightConstants.kTolerance;
  public final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.ApplyRobotSpeeds driveRequest;

  public CenterOnAprilTagCommand(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive) {
    this.drivetrain = driveTrain;
    this.driveRequest = new SwerveRequest.ApplyRobotSpeeds()
      //.withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1)
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
    LimelightCommand limelight = LimelightPolling.getInstance().limelights.get(Constants.LimelightConstants.limelightName);

    if (limelight.tid == -1) {
        System.out.println("CenterOnAprilTag: No April Tag Detected.");
        return;
    }

    Pose2d limelightEstimatedPose = LimelightHelpers.getBotPose2d(Constants.LimelightConstants.limelightName);
    Pose2d poseEstimatedPose = drivetrain.m_poseEstimator.getEstimatedPosition();

    SmartDashboard.putNumber("LL Est X", limelightEstimatedPose.getX());
    SmartDashboard.putNumber("LL Est Y", limelightEstimatedPose.getY());
    SmartDashboard.putNumber("Pose Estimator X", poseEstimatedPose.getX());
    SmartDashboard.putNumber("Pose Estimator Y", poseEstimatedPose.getY());

    SmartDashboard.putNumber("AprilTag TX", limelight.tx);
    SmartDashboard.putNumber("AprilTag TY", limelight.ty);

    ProfiledPIDController xController = new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(15.0, 3.0), 0.02);

    ProfiledPIDController yController = new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(15.0, 3.0), 0.02);

    ProfiledPIDController oController = new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(15.0, 3.0), 0.02);

    Pose2d targetPose = getRightDesiredPose(limelight.tid);
    double xSpeed = xController.calculate(poseEstimatedPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(poseEstimatedPose.getY(), targetPose.getY());
    double oSpeed = oController.calculate(poseEstimatedPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    drivetrain.driveApplySpeeds(xSpeed, ySpeed, oSpeed);
}

// Function to get the pose for right branches
private Pose2d getRightDesiredPose(double id) {
    switch ((int) id) {
        case 1: return PositionMap.coral1.getTargetPose();
        case 2: return PositionMap.coral3.getTargetPose();
        case 3: return PositionMap.processor.getTargetPose();
        case 4: return PositionMap.blueBarge.getTargetPose();
        case 5: return PositionMap.redBarge.getTargetPose();
        case 6: return PositionMap.L.getTargetPose();
        case 7: return PositionMap.B.getTargetPose();
        case 8: return PositionMap.D.getTargetPose();
        case 9: return PositionMap.F.getTargetPose();
        case 10: return PositionMap.H.getTargetPose();
        case 11: return PositionMap.J.getTargetPose();
        case 12: return PositionMap.coral1.getTargetPose();
        case 13: return PositionMap.coral3.getTargetPose();
        case 14: return PositionMap.redBarge.getTargetPose();
        case 15: return PositionMap.blueBarge.getTargetPose();
        case 16: return PositionMap.processor.getTargetPose();
        case 17: return PositionMap.D.getTargetPose();
        case 18: return PositionMap.B.getTargetPose();
        case 19: return PositionMap.L.getTargetPose();
        case 20: return PositionMap.J.getTargetPose();
        case 21: return PositionMap.H.getTargetPose();
        case 22: return PositionMap.F.getTargetPose();
        default: return PositionMap.OFF.getTargetPose();
    }
}

// Function to get the pose for left branches
private Pose2d leftRightDesiredPose(double id) {
    switch ((int) id) {
        case 1: return PositionMap.coral1.getTargetPose();
        case 2: return PositionMap.coral3.getTargetPose();
        case 3: return PositionMap.processor.getTargetPose();
        case 4: return PositionMap.blueBarge.getTargetPose();
        case 5: return PositionMap.redBarge.getTargetPose();
        case 6: return PositionMap.K.getTargetPose();
        case 7: return PositionMap.A.getTargetPose();
        case 8: return PositionMap.C.getTargetPose();
        case 9: return PositionMap.E.getTargetPose();
        case 10: return PositionMap.G.getTargetPose();
        case 11: return PositionMap.I.getTargetPose();
        case 12: return PositionMap.coral1.getTargetPose();
        case 13: return PositionMap.coral3.getTargetPose();
        case 14: return PositionMap.redBarge.getTargetPose();
        case 15: return PositionMap.blueBarge.getTargetPose();
        case 16: return PositionMap.processor.getTargetPose();
        case 17: return PositionMap.C.getTargetPose();
        case 18: return PositionMap.A.getTargetPose();
        case 19: return PositionMap.K.getTargetPose();
        case 20: return PositionMap.I.getTargetPose();
        case 21: return PositionMap.G.getTargetPose();
        case 22: return PositionMap.E.getTargetPose();
        default: return PositionMap.OFF.getTargetPose();
    }
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  //private double tx;
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (Math.abs(this.tx) <= this.kTolerance) {
      System.out.printf("CenterOnAprilTag: Lineup complete at TX %.5f", this.tx);
      System.out.println();
      return true;
    }*/
    return false; 
  }
}
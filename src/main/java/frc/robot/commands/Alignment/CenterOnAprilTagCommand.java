// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import edu.wpi.first.math.controller.PIDController;
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
  private final String lineupDirection;
  PIDController xController;
  PIDController yController;
  PIDController oController;

  public CenterOnAprilTagCommand(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive, String direction) {
    this.drivetrain = driveTrain;
    this.driveRequest = new SwerveRequest.ApplyRobotSpeeds()
      //.withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    this.lineupDirection = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tid =LimelightHelpers.getFiducialID(Constants.LimelightConstants.limelightName);
//double aprilTagId = LimelightHelpers.getFiducialID(Constants.LimelightConstants.limelightName);

    SmartDashboard.putNumber("AprilTag TID", tid);
    //SmartDashboard.putNumber("Last known TID", lastValidTid);

    if (tid == -1) {
        System.out.println("CenterOnAprilTag: No April Tag Detected.");
        return;
    }
    
    Pose2d poseEstimatedPose = drivetrain.m_poseEstimator.getEstimatedPosition();

    SmartDashboard.putNumber("Pose Estimator X", poseEstimatedPose.getX());
    SmartDashboard.putNumber("Pose Estimator Y", poseEstimatedPose.getY());

    xController = new PIDController(0.1, 0.00, 0.05);
    yController = new PIDController(0.1, 0.00, 0.05);
    oController = new PIDController(0.1, 0.00, 0.05);

    Pose2d targetPose = this.lineupDirection == "right" ? getRightDesiredPose((int)tid) : getLeftDesiredPose((int)tid);
    
    SmartDashboard.putNumber("TargetPoseX", targetPose.getX());
    SmartDashboard.putNumber("TargetPoseY", targetPose.getY());

    double xSpeed = xController.calculate(poseEstimatedPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(poseEstimatedPose.getY(), targetPose.getY());
    double oSpeed = oController.calculate(poseEstimatedPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    drivetrain.driveApplySpeeds(xSpeed, ySpeed, oSpeed);
}

  private Pose2d defaultPose = Constants.AlignmentConstants.B_RED;

  // Function to get the pose for right branches
  private Pose2d getRightDesiredPose(int aprilTagId) {
   // return Constants.AlignmentConstants.B_BLUE;
    switch (aprilTagId) {
      case 1:  return Constants.AlignmentConstants.CORAL1_RED;
      case 2:  return Constants.AlignmentConstants.CORAL3_RED;
      case 3:  return Constants.AlignmentConstants.PROCESSOR_RED;
      case 4:  return Constants.AlignmentConstants.BLUE_BARGE_RED;
      case 5:  return Constants.AlignmentConstants.RED_BARGE_RED;
      case 6:  return Constants.AlignmentConstants.L_RED;
      case 7:  return Constants.AlignmentConstants.B_RED;
      case 8:  return Constants.AlignmentConstants.D_RED;
      case 9:  return Constants.AlignmentConstants.F_RED;
      case 10: return Constants.AlignmentConstants.H_RED;
      case 11: return Constants.AlignmentConstants.J_RED;
      case 12: return Constants.AlignmentConstants.CORAL1_BLUE;
      case 13: return Constants.AlignmentConstants.CORAL3_BLUE;
      case 14: return Constants.AlignmentConstants.RED_BARGE_BLUE;
      case 15: return Constants.AlignmentConstants.BLUE_BARGE_BLUE;
      case 16: return Constants.AlignmentConstants.PROCESSOR_BLUE;
      case 17: return Constants.AlignmentConstants.D_BLUE;
      case 18: return Constants.AlignmentConstants.B_BLUE;
      case 19: return Constants.AlignmentConstants.L_BLUE;
      case 20: return Constants.AlignmentConstants.J_BLUE;
      case 21: return Constants.AlignmentConstants.H_BLUE;
      case 22: return Constants.AlignmentConstants.F_BLUE;
      default: return defaultPose;
  }
}

  // Function to get the pose for left branches
  private Pose2d getLeftDesiredPose(int aprilTagId) {
    switch (aprilTagId) {
      case 1:  return Constants.AlignmentConstants.CORAL1_RED;
      case 2:  return Constants.AlignmentConstants.CORAL3_RED;
      case 3:  return Constants.AlignmentConstants.PROCESSOR_RED;
      case 4:  return Constants.AlignmentConstants.BLUE_BARGE_RED;
      case 5:  return Constants.AlignmentConstants.RED_BARGE_RED;
      case 6:  return Constants.AlignmentConstants.K_RED;
      case 7:  return Constants.AlignmentConstants.A_RED;
      case 8:  return Constants.AlignmentConstants.C_RED;
      case 9:  return Constants.AlignmentConstants.E_RED;
      case 10: return Constants.AlignmentConstants.G_RED;
      case 11: return Constants.AlignmentConstants.I_RED;
      case 12: return Constants.AlignmentConstants.CORAL1_BLUE;
      case 13: return Constants.AlignmentConstants.CORAL3_BLUE;
      case 14: return Constants.AlignmentConstants.RED_BARGE_BLUE;
      case 15: return Constants.AlignmentConstants.BLUE_BARGE_BLUE;
      case 16: return Constants.AlignmentConstants.PROCESSOR_BLUE;
      case 17: return Constants.AlignmentConstants.C_BLUE;
      case 18: return Constants.AlignmentConstants.A_BLUE;
      case 19: return Constants.AlignmentConstants.K_BLUE;
      case 20: return Constants.AlignmentConstants.I_BLUE;
      case 21: return Constants.AlignmentConstants.G_BLUE;
      case 22: return Constants.AlignmentConstants.E_BLUE;
      default: return defaultPose;
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
package frc.robot.commands;

import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ChatGPTCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private static final double TARGET_OFFSET_TOLERANCE = 0.05; // Meters
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(2); // Radians
    private static final double ALIGN_SPEED = 0.5; // m/s
    double velocityX;
    double velocityY;
    double angularVelocity;

    public ChatGPTCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Align the robot to an AprilTag using Limelight vision.
     */
    public void initialize() {
        // Fetch the current pose from the Limelight
        LimelightHelpers.PoseEstimate tagPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (tagPose == null || tagPose.tagCount == 0) {
            SmartDashboard.putString("Alignment Status", "No AprilTag Detected");
            return;
        }

        // Desired target position (e.g., AprilTag's position)
        Pose2d targetPose = tagPose.pose;
        double targetX = targetPose.getX();
        double targetY = targetPose.getY();
        double targetYaw = targetPose.getRotation().getRadians();

        // Fetch current robot pose
        Pose2d robotPose = drivetrain.m_poseEstimator.getEstimatedPosition();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotYaw = robotPose.getRotation().getRadians();

        // Calculate position and angle error
        double errorX = targetX - robotX;
        double errorY = targetY - robotY;
        double angleError = targetYaw - robotYaw;

        // If within tolerances, stop movement
        if (Math.abs(errorX) < TARGET_OFFSET_TOLERANCE &&
            Math.abs(errorY) < TARGET_OFFSET_TOLERANCE &&
            Math.abs(angleError) < ANGLE_TOLERANCE) {
            drivetrain.stop();
            SmartDashboard.putString("Alignment Status", "Aligned with AprilTag");
            return;
        }

        // Calculate velocities to correct the pose
         velocityX = Math.copySign(ALIGN_SPEED, errorX);
         velocityY = Math.copySign(ALIGN_SPEED, errorY);
         angularVelocity = Math.copySign(ALIGN_SPEED, angleError);

        // Drive the robot to correct the errors
       
    }

    /**
     * Stop the robot.
     */

    @Override
    public void execute() {
        drivetrain.driveApplySpeeds(velocityX, velocityY, angularVelocity);
        SmartDashboard.putString("Alignment Status", "Aligning...");
        SmartDashboard.putString("Robot Pose", drivetrain.m_poseEstimator.getEstimatedPosition().toString());
    }
}
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain swerveDrive;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private Pose2d targetPose;

    public AutoAlignCommand(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        this.xController = new PIDController(2.0, 0.0, 0.3);  // X Translation Control
        this.yController = new PIDController(2.0, 0.0, 0.3);  // Y Translation Control
        this.thetaController = new PIDController(4.0, 0.0, 0.5);  // Rotation Control
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        // Check if the Limelight sees a tag
        if (LimelightHelpers.getTV("limelight")) { 
            double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");
            

            if (botPose.length >= 6) { // Ensure valid pose data
                double currentX = botPose[0];  // Robot's X position in target space
                double currentZ = botPose[2];  // Robot's Z position in target space
                double currentRotation = botPose[5];  // Yaw in degrees

                // Set the target position at (-0.5, 0.2) relative to the tag
                targetPose = new Pose2d(
                    new Translation2d(-0.5, 0.2),  
                    Rotation2d.fromDegrees(currentRotation) // Keep orientation
                );

                // Debugging: Print values to check correctness
                System.out.println("Current X: " + currentX + ", Current Z: " + currentZ);
                System.out.println("Target Pose -> X: " + targetPose.getX() + ", Z: " + targetPose.getY());
            } else {
                targetPose = swerveDrive.m_poseEstimator.getEstimatedPosition();
            }
        } else {
            targetPose = swerveDrive.m_poseEstimator.getEstimatedPosition();
        }
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveDrive.m_poseEstimator.getEstimatedPosition();
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(thetaSpeed)
            .withRotationalDeadband(0.1);
        
        swerveDrive.setControl(request);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveRequest.FieldCentric stopRequest = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);
        
        swerveDrive.setControl(stopRequest);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveToPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final String direction;
    private Pose2d targetPose;
    private Command pathCommand;

    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, String direction) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Starting DriveToPoseCommand...");
        updateTargetPose();
        startPath();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute(); 
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.cancel();
        System.out.println("DriveToPoseCommand finished.");
    }

    /** Updates the target pose dynamically based on AprilTag ID */
    private void updateTargetPose() {
        double aprilTagId = LimelightHelpers.getFiducialID(Constants.LimelightConstants.limelightName);

        if (aprilTagId == -1) {
            System.out.println("No valid AprilTag detected. Defaulting to A_BLUE.");
            targetPose = Constants.AlignmentConstants.A_BLUE;
        } else {
            targetPose = getTargetPose((int) aprilTagId);
        }

        // Flip pose if we're on the red alliance
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) targetPose = FlippingUtil.flipFieldPose(targetPose);
        
    }

    /** Creates and follows a smooth path to the target pose */
    private void startPath() {

        if (targetPose != null) {
            System.out.println("Driving to: " + targetPose);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivetrain.m_poseEstimator.getEstimatedPosition(), targetPose);

            // Generate a smooth path from the robot's current pose to targetPose
            PathPlannerPath generatedPath = new PathPlannerPath(waypoints, 
                Constants.PathplannerConstants.constraints, null, 
                new GoalEndState(0, targetPose.getRotation())); 
            
            //Prevents flipping of the current pose
            generatedPath.preventFlipping = true;

            // Follow the dynamically created path
            pathCommand = AutoBuilder.followPath(generatedPath);

            // Instead of scheduling it, call `execute()` manually
            pathCommand.initialize();
        }
    }

    /** Returns the correct target pose based on AprilTag ID and direction */
    private Pose2d getTargetPose(int aprilTagId) {
        return switch (direction) {
            case "left" -> switch (aprilTagId) {
                case 18, 7 -> Constants.AlignmentConstants.A_BLUE;
                case 19, 6 -> Constants.AlignmentConstants.K_BLUE;
                case 20, 11 -> Constants.AlignmentConstants.I_BLUE;
                case 21, 10 -> Constants.AlignmentConstants.G_BLUE;
                case 22, 9 -> Constants.AlignmentConstants.E_BLUE;
                case 17, 8 -> Constants.AlignmentConstants.C_BLUE;
                default -> {
                    System.out.println("Unknown AprilTag ID for left: " + aprilTagId);
                    yield Constants.AlignmentConstants.A_BLUE;
                }
            };
            case "right" -> switch (aprilTagId) {
                case 18, 7 -> Constants.AlignmentConstants.B_BLUE;
                case 19, 6 -> Constants.AlignmentConstants.L_BLUE;
                case 20, 11 -> Constants.AlignmentConstants.J_BLUE;
                case 21, 10 -> Constants.AlignmentConstants.H_BLUE;
                case 22, 9 -> Constants.AlignmentConstants.F_BLUE;
                case 17, 8 -> Constants.AlignmentConstants.D_BLUE;
                default -> {
                    System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                    yield Constants.AlignmentConstants.B_BLUE;
                }
            };
            default -> Constants.AlignmentConstants.A_BLUE;
        };
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PathplannerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.LimelightHelpers;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveToPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final String direction;
    private Pose2d targetPose;
    private Pose2d cache;
    private Command pathCommand;
    private boolean isBarge;

    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, String direction) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isBarge = direction.equals("MidBarge") || direction.equals("LeftBarge") || direction.equals("RightBarge");
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
        if (pathCommand != null) {
            pathCommand.cancel();
            System.out.println("DriveToPoseCommand finished.");
            }
    }

    /** Updates the target pose dynamically based on AprilTag ID */
    private void updateTargetPose() {

        if (targetPose != null && drivetrain.m_poseEstimator.getEstimatedPosition().equals(targetPose)) {
            System.out.println("Already at target pose. No path needed.");
            pathCommand = null;  // Ensure we don't try to execute a null command
            return;
        }
        
        double aprilTagId = LimelightHelpers.getFiducialID(Constants.LimelightConstants.lowerLimelightName);
        cache = getTargetPose((int) drivetrain.aprilTagCache);
        
        if (aprilTagId == -1) {
            System.out.println("No valid AprilTag detected. Defaulting to last tag seen");
            targetPose = cache;
        } else {
            targetPose = getTargetPose((int) aprilTagId);
        }

        // Flip pose if we're on the red alliance
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) targetPose = FlippingUtil.flipFieldPose(targetPose);
        
    }

    private void startPath() {
        if (targetPose != null) {
            Pose2d currentPose = drivetrain.m_poseEstimator.getEstimatedPosition();
            
            // Check if the robot is already at the target position within a small tolerance
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double angleDifference = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees());
    
            if (distance < 0.05 && angleDifference < 2) { // 5 cm and 2 degrees tolerance
                System.out.println("Already at target pose. No path needed.");
                pathCommand = null;
                return;
            }
            
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
    
            if (waypoints.isEmpty()) {
                System.out.println("No waypoints generated. Skipping path.");
                pathCommand = null;
                return;
            }

            if (isBarge) {
                PathPlannerPath bargePath = getBargePath();
                
                    if (bargePath == null) {
                        System.out.println("Error: Barge path is null. Skipping execution.");
                        pathCommand = null;
                        return; 
                    }
                //if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) bargePath = bargePath.flipPath();
                pathCommand = AutoBuilder.pathfindThenFollowPath(bargePath, Constants.PathplannerConstants.constraints);
            } else {
                PathPlannerPath generatedPath = new PathPlannerPath(waypoints, 
                Constants.PathplannerConstants.constraints, null, 
                new GoalEndState(0, targetPose.getRotation()));
                generatedPath.preventFlipping = true;
                pathCommand = AutoBuilder.followPath(generatedPath);
            }

            if (pathCommand == null) {
                System.out.println("PathPlanner failed to generate a command. Skipping execution.");
                return;
            }
    
            try {
                pathCommand.initialize();
            } catch (Exception e) {
                e.printStackTrace();
                return;
            }
        }
    }

    /** Returns the correct target pose based on AprilTag ID and direction */
    private Pose2d getTargetPose(int aprilTagId) {

        return switch (direction) {
            case "Left" -> switch (aprilTagId) {
                case 18, 7 -> Constants.AlignmentConstants.A_BLUE;
                case 19, 6 -> Constants.AlignmentConstants.K_BLUE;
                case 20, 11 -> Constants.AlignmentConstants.I_BLUE;
                case 21, 10 -> Constants.AlignmentConstants.G_BLUE;
                case 22, 9 -> Constants.AlignmentConstants.E_BLUE;
                case 17, 8 -> Constants.AlignmentConstants.C_BLUE;
                case 12 -> Constants.AlignmentConstants.CORAL1LEFT;
                case 13 -> Constants.AlignmentConstants.CORAL3LEFT;
                // case 14, 5 -> Constants.AlignmentConstants.BARGELEFT;
                case 3, 16 -> Constants.AlignmentConstants.PROCESSOR;
                default -> {
                    System.out.println("Unknown AprilTag ID for left: " + aprilTagId);
                    yield Constants.AlignmentConstants.A_BLUE;
                }
            };
            case "Right" -> switch (aprilTagId) {
                case 18, 7 -> Constants.AlignmentConstants.B_BLUE;
                case 19, 6 -> Constants.AlignmentConstants.L_BLUE;
                case 20, 11 -> Constants.AlignmentConstants.J_BLUE;
                case 21, 10 -> Constants.AlignmentConstants.H_BLUE;
                case 22, 9 -> Constants.AlignmentConstants.F_BLUE;
                case 17, 8 -> Constants.AlignmentConstants.D_BLUE;
                case 12 -> Constants.AlignmentConstants.CORAL1RIGHT;
                case 13 -> Constants.AlignmentConstants.CORAL3RIGHT;
                // case 14, 5 -> Constants.AlignmentConstants.BARGERIGHT;
                case 3, 16 -> Constants.AlignmentConstants.PROCESSOR;
                default -> {
                    System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                    yield Constants.AlignmentConstants.B_BLUE;
                }
            };
            default -> Constants.AlignmentConstants.A_BLUE;
        };
    }
   private PathPlannerPath getBargePath(){
    try {
        return switch (direction) {
            case "MidBarge" -> PathPlannerPath.fromPathFile("MidBarge");
            case "LeftBarge" -> PathPlannerPath.fromPathFile("LeftBarge");
            case "RightBarge" -> PathPlannerPath.fromPathFile("RightBarge");
            default -> throw new IllegalStateException("Invalid barge direction: " + direction);
        };
    } catch (Exception e) {
        System.err.println("Error loading PathPlanner path for direction: " + direction);
        e.printStackTrace();
        return null;
    }
}
}
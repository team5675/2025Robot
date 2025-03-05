package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.RumbleCommand;
import frc.robot.subsystems.Coral.Coral;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.List;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveToPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final String direction;
    private Pose2d targetPose;
    private Pose2d cache;
    private Command pathCommand;
    private boolean isBarge;
    private double aprilTagId;
    private final Supplier<Boolean> useReefTagsSupplier;
    private boolean isAlgae;

    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, String direction, Supplier<Boolean> useReefTagsSupplier, boolean isAlgae) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        this.useReefTagsSupplier = useReefTagsSupplier;
        this.isAlgae = isAlgae;
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
            new RumbleCommand().schedule();
            }
    }

    /** Updates the target pose dynamically based on AprilTag ID */
    private void updateTargetPose() {
        
        if (targetPose != null && drivetrain.m_poseEstimator.getEstimatedPosition().equals(targetPose)) {
            System.out.println("Already at target pose. No path needed.");
            pathCommand = null; 
            return;
        }
        if ((!useReefTagsSupplier.get() || !Coral.getInstance().bb2Tripped) && !isAlgae) {  
            aprilTagId = LimelightHelpers.getFiducialID(Constants.LimelightConstants.upperLimelightName);
        } else {
            aprilTagId = LimelightHelpers.getFiducialID(Constants.LimelightConstants.lowerLimelightName);
        }

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
    
            if (distance < 0.05 && angleDifference < 0.7) { // 5 cm and 0.7 degrees tolerance
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
                case 18, 7 -> AlignmentConstants.REEF_A;
                case 19, 6 -> AlignmentConstants.REEF_K;
                case 20, 11 -> AlignmentConstants.REEF_I;
                case 21, 10 -> AlignmentConstants.REEF_G;
                case 22, 9 -> AlignmentConstants.REEF_E;
                case 17, 8 -> AlignmentConstants.REEF_C;
                case 12, 2 -> AlignmentConstants.CORAL1LEFT;
                case 13, 1 -> AlignmentConstants.CORAL3LEFT;
                // case 3, 16 -> AlignmentConstants.PROCESSOR;
                default -> {
                    System.out.println("Unknown AprilTag ID for left: " + aprilTagId);
                    yield drivetrain.m_poseEstimator.getEstimatedPosition();
                }
            };
            case "Right" -> switch (aprilTagId) {
                case 18, 7 -> AlignmentConstants.REEF_B;
                case 19, 6 -> AlignmentConstants.REEF_L;
                case 20, 11 -> AlignmentConstants.REEF_J;
                case 21, 10 -> AlignmentConstants.REEF_H;
                case 22, 9 -> AlignmentConstants.REEF_F;
                case 17, 8 -> AlignmentConstants.REEF_D;
                case 12, 2 -> AlignmentConstants.CORAL1RIGHT;
                case 13, 1 -> AlignmentConstants.CORAL3RIGHT;
                // case 3, 16 -> AlignmentConstants.PROCESSOR;
                default -> {
                    System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                    yield drivetrain.m_poseEstimator.getEstimatedPosition();
                }
            };
            case "Algae" -> switch (aprilTagId) {
                case 18, 7 -> AlignmentConstants.ALGAE_AB;
                case 19, 6 -> AlignmentConstants.ALGAE_KL;
                case 20, 11 -> AlignmentConstants.ALGAE_IJ;
                case 21, 10 -> AlignmentConstants.ALGAE_GH;
                case 22, 9 -> AlignmentConstants.ALGAE_EF;
                case 17, 8 -> AlignmentConstants.ALGAE_CD;
                default -> {
                    System.out.println("Unknown AprilTag ID for algae: " + aprilTagId);
                    yield drivetrain.m_poseEstimator.getEstimatedPosition();
                }
            };
            default -> drivetrain.m_poseEstimator.getEstimatedPosition();
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
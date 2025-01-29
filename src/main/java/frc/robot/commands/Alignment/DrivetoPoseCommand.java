package frc.robot.commands.Alignment;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DrivetoPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final String direction;
    private Pose2d pose;
    private Command pathCommand; // Store the generated path command

    /** Creates a new DriveToPoseCommand for Teleop Use. */
    public DrivetoPoseCommand(CommandSwerveDrivetrain drivetrain, String direction) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        addRequirements(drivetrain); // Ensure this command owns the drivetrain subsystem
    }

    @Override
    public void initialize() {
        System.out.println("DriveToPoseCommand started...");

        // Get the AprilTag ID from Limelight
        double aprilTagId = LimelightHelpers.getFiducialID(Constants.LimelightConstants.limelightName);
        System.out.println("Detected AprilTag ID: " + aprilTagId);

       
            if (direction.equals("left")) {
                switch ((int) aprilTagId) {
                    case 3: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 4: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 5: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 6: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 7: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 8: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 9: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 10: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 13: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 14: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 16: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 17: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 18: pose = Constants.AlignmentConstants.A_BLUE; break;
                    default:
                        pose = Constants.AlignmentConstants.A_BLUE;
                        System.out.println("Unknown AprilTag ID for left: " + aprilTagId);
                        break;
                }
            } else { 
                switch ((int) aprilTagId) {
                    case 3: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 4: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 5: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 6: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 7: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 8: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 9: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 10: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 13: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 14: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 16: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 17: pose = Constants.AlignmentConstants.A_BLUE; break;
                    case 18: pose = Constants.AlignmentConstants.B_BLUE; break;
                    default:
                        pose = Constants.AlignmentConstants.B_BLUE;
                        System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                        break;
                }
            }
        

        System.out.println("Pose set to: " + pose);

        // Generate and schedule the path command every time the command starts
        pathCommand = AutoBuilder.pathfindToPose(pose, Constants.PathplannerConstants.constraints, 0.0);
        if (pathCommand != null) {
            pathCommand.schedule(); // Runs the path command asynchronously
            System.out.println("Pathfinding command scheduled.");
        } else {
            System.out.println("Error: Pathfinding command is null.");
        }
    }

    @Override
    public boolean isFinished() {
        // Allow command to restart when the button is pressed again
        return false;
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("DriveToPoseCommand ending. Interrupted: " + interrupted);

      // Cancel the path command if it's still running
    
          pathCommand.cancel(); // 💡 Explicitly cancel path command when button is released
          System.out.println("Path command canceled.");
      

      // Immediately stop the drivetrain when the button is released
      
      System.out.println("Drivetrain stopped.");

      pathCommand = null;
    }
}

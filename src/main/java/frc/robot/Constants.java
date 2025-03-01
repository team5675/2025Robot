package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

    public class LimelightConstants {
        public static String lowerLimelightName = "limelight-lower";
        public static String upperLimelightName = "limelight-upper";
        public static double kTolerance = 0.2;
        public static double minStrafe = 0.3;
        public static double limelightForward = 0;
        public static double limelightSide = 0.00635;
        public static double limelightUp = 0.2254;
        public static double limelightRoll= 0.0;
        public static double limelightPitch = 6.0;
        public static double limelightYaw = 0.0;
    }
    
    public class PathplannerConstants {
        // Create the constraints to use while pathfinding
        public static final PathConstraints constraints = new PathConstraints(
        2.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540));
    }
}
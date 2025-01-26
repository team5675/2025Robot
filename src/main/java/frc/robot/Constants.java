package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

    public class LimelightConstants {
        public static String limelightName = "limelight";
        public static double kTolerance = 0.2;
        public static double minStrafe = 0.3;
    }
    
    public class PathplannerConstants {
        // Create the constraints to use while pathfinding
        public static final PathConstraints constraints = new PathConstraints(
        5.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
    }
    public class AlignmentConstants {

        // Rotation constants
        private static final Rotation2d DEFAULT_ROTATION = new Rotation2d(0);
        private static final Rotation2d REEF_L_ROTATION = new Rotation2d(-56.224);

        // Pose constants
        public static final Pose2d OFF_BLUE = new Pose2d(1.0, 1.0, DEFAULT_ROTATION);
        public static final Pose2d OFF_RED = new Pose2d();

        public static final Pose2d A_BLUE = new Pose2d(3.1, 4.19, DEFAULT_ROTATION);
        public static final Pose2d A_RED = new Pose2d(1.235, 7.045, DEFAULT_ROTATION);

        public static final Pose2d B_BLUE = new Pose2d(3.1, 3.83, DEFAULT_ROTATION);
        public static final Pose2d B_RED = new Pose2d();

        public static final Pose2d C_BLUE = new Pose2d(4.07, 3.306, DEFAULT_ROTATION);
        public static final Pose2d C_RED = new Pose2d();

        public static final Pose2d D_BLUE = new Pose2d(1.0, 1.0, DEFAULT_ROTATION);
        public static final Pose2d D_RED = new Pose2d();

        public static final Pose2d E_BLUE = new Pose2d(4.904, 3.306, DEFAULT_ROTATION);
        public static final Pose2d E_RED = new Pose2d();

        public static final Pose2d F_BLUE = new Pose2d(1.0, 1.0, DEFAULT_ROTATION);
        public static final Pose2d F_RED = new Pose2d();

        public static final Pose2d G_BLUE = new Pose2d(5.32, 4.02, DEFAULT_ROTATION);
        public static final Pose2d G_RED = new Pose2d();

        public static final Pose2d H_BLUE = new Pose2d(1.0, 1.0, DEFAULT_ROTATION);
        public static final Pose2d H_RED = new Pose2d();

        public static final Pose2d I_BLUE = new Pose2d(4.904, 4.47, DEFAULT_ROTATION);
        public static final Pose2d I_RED = new Pose2d();

        public static final Pose2d J_BLUE = new Pose2d(1.0, 1.0, DEFAULT_ROTATION);
        public static final Pose2d J_RED = new Pose2d();

        public static final Pose2d K_BLUE = new Pose2d(4.07, 4.745, DEFAULT_ROTATION);
        public static final Pose2d K_RED = new Pose2d();

        public static final Pose2d L_BLUE = new Pose2d(13.8, 2.5, REEF_L_ROTATION);
        public static final Pose2d L_RED = new Pose2d();

        public static final Pose2d CORAL1_BLUE = new Pose2d(1.217, 7.045, DEFAULT_ROTATION);
        public static final Pose2d CORAL1_RED = new Pose2d();

        public static final Pose2d CORAL3_BLUE = new Pose2d();
        public static final Pose2d CORAL3_RED = new Pose2d();

        public static final Pose2d RED_BARGE_BLUE = new Pose2d();
        public static final Pose2d RED_BARGE_RED = new Pose2d();

        public static final Pose2d BLUE_BARGE_BLUE = new Pose2d();
        public static final Pose2d BLUE_BARGE_RED = new Pose2d();

        public static final Pose2d PROCESSOR_BLUE = new Pose2d();
        public static final Pose2d PROCESSOR_RED = new Pose2d();
    }
}
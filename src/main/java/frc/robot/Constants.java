package frc.robot;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;


public class Constants {

    public class SwerveConstants {

        public static final double maxSwerveSpeedMS = Units.feetToMeters(17.6);
        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

        public static final double XboxJoystickDeadband = 0.01;
        public static final int kDriverControllerPort = 0;

        public static final double VelocityControllerLoopTime = 0.13;//seconds
        public static final double RobotMass = Units.lbsToKilograms(150);
        

    }

    public class LimelightConstants {

        public static final String limelightName = "limelight";

        //Relative to center of robot on the floor
        public static final Pose3d limelightPhysicalLocation = new Pose3d(0.3492, 0.3746, 0.635, new Rotation3d(0, -12, 0));

        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, 0.001);
        public static final Matrix<N3, N1> driveMeasurementStdDevs = VecBuilder.fill(0.17, 0.17, 0.001);

    }

    public class LEDConstants{
        public static final int blinkinID = 1;
        public static final double LEDRainbow = -0.99;
        public static final double LEDOceanRainbow = -0.95;
        public static final double LEDOrange = 0.65;
        public static final double LEDBlue = 0.87;
        public static final double LEDRed = 0.59;
        //Change the default pattern by holding the mode button, then holding the other button
    }
    /**
     * All constants assume Blue side origin, and are in meters
     */
    public class FieldConstantsDep {

        /* Field Relative Plane
         *         2                                        5
         *  --------------------------------------------------------
         *  |                      Y                               |
         *  |                      ^                               |
         *1 |                      |                               | 4
         *  |              3      Z.->X               6            |
         *  |                                                      |
         *  |                                                      |
         *  |                                                      |
         *  ---------------------------------------------------------
         * 1-Blue side Speaker
         * 2-Blue side Amp
         * 3-Blue side Trap
         */
    }
}

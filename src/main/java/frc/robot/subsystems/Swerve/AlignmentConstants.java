package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {
    // Rotation constants
    // Blue letters come first
    private static final Rotation2d AB = new Rotation2d(Math.toRadians(0));
    private static final Rotation2d CD = new Rotation2d(Math.toRadians(60));
    private static final Rotation2d EF = new Rotation2d(Math.toRadians(120));
    private static final Rotation2d GH = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d IJ = new Rotation2d(Math.toRadians(240));
    private static final Rotation2d KL = new Rotation2d(Math.toRadians(300));
    private static final Rotation2d RIGHTCORALSTATION = new Rotation2d(Math.toRadians(54));
    private static final Rotation2d LEFTCORALSTATION = new Rotation2d(Math.toRadians(-54));
    private static final Rotation2d BARGE = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d PROCESSORANGLE = new Rotation2d(Math.toRadians(-90));

    //Home Locations
    // public static final Pose2d REEF_A = new Pose2d(3.213, 4.204, AB);
   
    // public static final Pose2d REEF_B = new Pose2d(3.245, 3.821, AB);
   
    // public static final Pose2d REEF_C = new Pose2d(3.695, 3.008, CD);
   
    // public static final Pose2d REEF_D = new Pose2d(4.005, 2.862, CD);
   
    // public static final Pose2d REEF_E = new Pose2d(4.970, 2.839, EF);
    
    // public static final Pose2d REEF_F = new Pose2d(5.265, 3.037, EF);
    
    // public static final Pose2d REEF_G = new Pose2d(5.762, 3.851, GH);
    
    // public static final Pose2d REEF_H = new Pose2d(5.73, 4.21, GH);
    
    // public static final Pose2d REEF_I = new Pose2d(5.274, 5.037, IJ);
    
    // public static final Pose2d REEF_J = new Pose2d(4.939, 5.198, IJ);
    
    // public static final Pose2d REEF_K = new Pose2d(3.989, 5.201, KL);
    
    // public static final Pose2d REEF_L = new Pose2d(3.707, 5.010, KL);

    //St Joe Locations
    public static final Pose2d REEF_A = new Pose2d(3.237, 4.199, AB);
   
    public static final Pose2d REEF_B = new Pose2d(3.2606, 3.8572, AB);
    //Changed C
    public static final Pose2d REEF_C = new Pose2d(3.72599, 3.02979, CD);
    //D works
    public static final Pose2d REEF_D = new Pose2d(4.021, 2.8778, CD);
   
    public static final Pose2d REEF_E = new Pose2d(4.9572, 2.849, EF);
    //GVSU was 5.256, 3.0469
    public static final Pose2d REEF_F = new Pose2d(5.23, 3.025, EF);
    
    public static final Pose2d REEF_G = new Pose2d(5.7419, 3.853, GH);
    
    public static final Pose2d REEF_H = new Pose2d(5.7217, 4.19517, GH);
    
    public static final Pose2d REEF_I = new Pose2d(5.2662, 5.0143, IJ);
    //J works
    public static final Pose2d REEF_J = new Pose2d(4.9607, 5.1771, IJ);
    //
    public static final Pose2d REEF_K = new Pose2d(4.00304, 5.1913, KL);
    
    public static final Pose2d REEF_L = new Pose2d(3.7185, 4.993, KL);
    
    public static final Pose2d CORAL1RIGHT = new Pose2d(1.536, 0.726, RIGHTCORALSTATION);
    public static final Pose2d CORAL1LEFT = new Pose2d(0.800, 1.258, RIGHTCORALSTATION);

    public static final Pose2d CORAL3RIGHT = new Pose2d(1.536, 7.277, LEFTCORALSTATION);
    public static final Pose2d CORAL3LEFT = new Pose2d(0.81, 6.802, LEFTCORALSTATION);

    public static final Pose2d BARGERIGHT = new Pose2d(8.510, 5.074, BARGE);
    public static final Pose2d BARGECENTER = new Pose2d(8.510, 6.168, BARGE);
    public static final Pose2d BARGELEFT = new Pose2d(8.510, 7.228, BARGE);
    
    public static final Pose2d PROCESSOR = new Pose2d(5.979, 0.541, PROCESSORANGLE);

    //Home Locations
    // public static final Pose2d ALGAE_AB = new Pose2d(3.1876, 4.026, AB);

    // public static final Pose2d ALGAE_CD = new Pose2d(3.839, 2.899, CD);

    // public static final Pose2d ALGAE_EF = new Pose2d(5.1396, 2.899, EF);

    // public static final Pose2d ALGAE_GH = new Pose2d(5.7909, 4.0258, GH);

    // public static final Pose2d ALGAE_IJ = new Pose2d(5.1396, 5.15242, IJ);
    
    // public static final Pose2d ALGAE_KL = new Pose2d(3.8389, 5.1524, KL);

    // Algae center positions St. Joe
    public static final Pose2d ALGAE_AB = new Pose2d(3.2488, 4.0281, AB);

    public static final Pose2d ALGAE_CD = new Pose2d(3.8719, 2.9510, CD);

    public static final Pose2d ALGAE_EF = new Pose2d(5.1066, 2.9479, EF);

    public static final Pose2d ALGAE_GH = new Pose2d(5.7318, 4.0241, GH);

    public static final Pose2d ALGAE_IJ = new Pose2d(5.1102, 5.10225, IJ);
    
    public static final Pose2d ALGAE_KL = new Pose2d(3.86077, 5.109215, KL);

}
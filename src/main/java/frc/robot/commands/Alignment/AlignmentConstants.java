package frc.robot.commands.Alignment;

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
    private static final Rotation2d RIGHTCORALSTATION = new Rotation2d(Math.toRadians(53));
    private static final Rotation2d LEFTCORALSTATION = new Rotation2d(Math.toRadians(-53));
    private static final Rotation2d BARGE = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d PROCESSORANGLE = new Rotation2d(Math.toRadians(-90));

    // Corrected Pose2d values (same X and Y, updated rotation)
    public static final Pose2d A_BLUE = new Pose2d(3.171, 4.19, AB);
    public static final Pose2d A_RED = new Pose2d(14.381, 3.862, AB);

    public static final Pose2d B_BLUE = new Pose2d(3.171, 3.83, AB);
    public static final Pose2d B_RED = new Pose2d(14.386, 4.163, AB);

    public static final Pose2d C_BLUE = new Pose2d(3.662, 2.963, CD);
    public static final Pose2d C_RED = new Pose2d(13.871, 5.079, CD);

    public static final Pose2d D_BLUE = new Pose2d(3.9, 2.818, CD);
    public static final Pose2d D_RED = new Pose2d(13.579, 5.231, CD);

    public static final Pose2d E_BLUE = new Pose2d(5.03, 2.828, EF);
    public static final Pose2d E_RED = new Pose2d(12.531, 5.229, EF);

    public static final Pose2d F_BLUE = new Pose2d(5.320, 2.963, EF);
    public static final Pose2d F_RED = new Pose2d(12.261, 5.076, EF);

    public static final Pose2d G_BLUE = new Pose2d(5.807, 3.833, GH);
    public static final Pose2d G_RED = new Pose2d(11.750, 4.183, GH);

    public static final Pose2d H_BLUE = new Pose2d(5.807, 4.175, GH);
    public static final Pose2d H_RED = new Pose2d(11.750, 3.858, GH);

    public static final Pose2d I_BLUE = new Pose2d(5.289, 5.077, IJ);
    public static final Pose2d I_RED = new Pose2d(12.258, 2.971, IJ);

    public static final Pose2d J_BLUE = new Pose2d(5.019, 5.232, IJ);
    public static final Pose2d J_RED = new Pose2d(12.544, 2.802, IJ);

    public static final Pose2d K_BLUE = new Pose2d(3.969, 5.197, KL);
    public static final Pose2d K_RED = new Pose2d(13.564, 2.804, KL);

    public static final Pose2d L_BLUE = new Pose2d(3.729, 5.087, KL);
    public static final Pose2d L_RED = new Pose2d(13.564, 2.804, KL);

    public static final Pose2d CORAL1RIGHT = new Pose2d(1.536, 0.726, RIGHTCORALSTATION);
    public static final Pose2d CORAL1LEFT = new Pose2d(0.800, 1.258, RIGHTCORALSTATION);

    public static final Pose2d CORAL3RIGHT = new Pose2d(1.536, 7.277, LEFTCORALSTATION);
    public static final Pose2d CORAL3LEFT = new Pose2d(0.81, 6.802, LEFTCORALSTATION);

    public static final Pose2d BARGERIGHT = new Pose2d(8.510, 5.074, BARGE);
    public static final Pose2d BARGECENTER = new Pose2d(8.510, 6.168, BARGE);
    public static final Pose2d BARGELEFT = new Pose2d(8.510, 7.228, BARGE);

    public static final Pose2d PROCESSOR = new Pose2d(5.979, 0.541, PROCESSORANGLE);

    // Algae center positions
    public static final Pose2d ALGAE_1_RED = new Pose2d();
    public static final Pose2d ALGAE_1_BLUE = new Pose2d();

    public static final Pose2d ALGAE_2_RED = new Pose2d();
    public static final Pose2d ALGAE_2_BLUE = new Pose2d();

    public static final Pose2d ALGAE_3_RED = new Pose2d();
    public static final Pose2d ALGAE_3_BLUE = new Pose2d();

    public static final Pose2d ALGAE_4_RED = new Pose2d();
    public static final Pose2d ALGAE_4_BLUE = new Pose2d();

    public static final Pose2d ALGAE_5_RED = new Pose2d();
    public static final Pose2d ALGAE_5_BLUE = new Pose2d();

    public static final Pose2d ALGAE_6_RED = new Pose2d();
    public static final Pose2d ALGAE_6_BLUE = new Pose2d();
}
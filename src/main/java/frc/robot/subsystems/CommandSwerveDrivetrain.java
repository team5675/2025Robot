package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import edu.wpi.first.math.estimator.*;
import frc.robot.*;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private boolean useAprilTagUpdates = true;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.configAutoBuilder();

        // SwerveDriveKinematics
        // Rotation2d
        // SwerveModulePosition[]
        // Pose2d
        this.m_poseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(),
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions,
            this.getState().Pose
        );
        
        this.m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.configAutoBuilder();
        
        // SwerveDriveKinematics
        // Rotation2d
        // SwerveModulePosition[]
        // Pose2d
        this.m_poseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(),
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions,
            this.getState().Pose
        );

        this.m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.configAutoBuilder();

        // SwerveDriveKinematics
        // Rotation2d
        // SwerveModulePosition[]
        // Pose2d
        this.m_poseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(),
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions,
            this.getState().Pose
        );

        this.m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public SwerveDrivePoseEstimator m_poseEstimator;
    public Field2d m_field;

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        m_poseEstimator.update(this.getPigeon2().getRotation2d(), this.getState().ModulePositions);
        //Pose Estimation using AprilTags
        LimelightHelpers.SetRobotOrientation(
            Constants.LimelightConstants.limelightName,
            this.getPigeon2().getYaw().getValueAsDouble(),
            0, 0, 0, 0, 0
        );
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimelightConstants.limelightName);

        if (mt2 == null) {
            return;
        }
        
        if (!(mt2.tagCount == 0)) {
        // If there is an april tag
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,0.7));
            m_poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds
            );
            
        }
        
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        //Debug Values
        SmartDashboard.putNumber("Robot Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Limelight TID", LimelightHelpers.getLimelightNTDouble(Constants.LimelightConstants.limelightName, "tid"));
        SmartDashboard.putString("Limelight Pose", mt2.pose.toString());
    }

    public void configAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.m_poseEstimator.getEstimatedPosition(),   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(5, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(3, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public void driveApplySpeeds(double xVelocity, double yVelocity, double angularVelocity) {
        this.setControl(
            new SwerveRequest.FieldCentric()
                //.withDeadband(DEADBAND)
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(angularVelocity)
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
        );
    }
    public Command driveToPose(CommandSwerveDrivetrain drivetrain, String direction) {
        Pose2d pose = null;
        double aprilTagId = LimelightHelpers.getFiducialID(Constants.LimelightConstants.limelightName);

        if (aprilTagId == -1) {
            System.out.println("\n\nWarning: No valid AprilTag detected. Defaulting to A_BLUE.");
        }

        if (direction.equals("left")) {
            switch ((int) aprilTagId) {
                // BLUE SIDE - LEFT POSITIONS (AprilTags 17-22)
                case 18, 7: pose = Constants.AlignmentConstants.A_BLUE; break;
                case 19, 6: pose = Constants.AlignmentConstants.K_BLUE; break;
                case 20, 11: pose = Constants.AlignmentConstants.I_BLUE; break;
                case 21, 10: pose = Constants.AlignmentConstants.G_BLUE; break;
                case 22, 9: pose = Constants.AlignmentConstants.E_BLUE; break;
                case 17, 8: pose = Constants.AlignmentConstants.C_BLUE; break;
        
                // RED SIDE - LEFT POSITIONS (AprilTags 6-11)
                // case 7: pose = Constants.AlignmentConstants.A_RED; break;
                // case 8: pose = Constants.AlignmentConstants.C_RED; break;
                // case 9: pose = Constants.AlignmentConstants.E_RED; break;
                // case 10: pose = Constants.AlignmentConstants.G_RED; break;
                // case 11: pose = Constants.AlignmentConstants.I_RED; break;
                // case 6: pose = Constants.AlignmentConstants.K_RED; break;
        
                default:
                    pose = Constants.AlignmentConstants.A_BLUE;
                    System.out.println( "\n\nDefaulting to A_BLUE.");
                    System.out.println("Unknown AprilTag ID for left: " + aprilTagId);
                    break;
            }
        } else { 
            switch ((int) aprilTagId) {
                // BLUE SIDE - RIGHT POSITIONS (AprilTags 17-22)
                case 18, 7: pose = Constants.AlignmentConstants.B_BLUE; break;
                case 19, 6: pose = Constants.AlignmentConstants.L_BLUE; break;
                case 20, 11: pose = Constants.AlignmentConstants.J_BLUE; break;
                case 21, 10: pose = Constants.AlignmentConstants.H_BLUE; break;
                case 22, 9: pose = Constants.AlignmentConstants.F_BLUE; break;
                case 17, 8: pose = Constants.AlignmentConstants.D_BLUE; break;
        
                // RED SIDE - RIGHT POSITIONS (AprilTags 6-11)
                // case 7: pose = Constants.AlignmentConstants.B_RED; break;
                // case 8: pose = Constants.AlignmentConstants.D_RED; break;
                // case 9: pose = Constants.AlignmentConstants.F_RED; break;
                // case 10: pose = Constants.AlignmentConstants.H_RED; break;
                // case 11: pose = Constants.AlignmentConstants.J_RED; break;
                // case 6: pose = Constants.AlignmentConstants.L_RED; break;
        
                default:
                    pose = Constants.AlignmentConstants.B_BLUE;
                    System.out.println( "\n\nDefaulting to B_BLUE.");
                    System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                    break;
            }
        }

        if (pose == null) {
            System.out.println("\n\nError: No valid target pose. Aborting path.");
            return Commands.none();  // Prevents execution of an invalid path
        }
        System.out.println("\n\n\nPose"+ pose.toString());

        return AutoBuilder.pathfindToPoseFlipped(pose, Constants.PathplannerConstants.constraints, 0.0);
        
    }

    public void setUseAprilTagUpdates(boolean use) {
        useAprilTagUpdates = use;
    }
    
    public boolean shouldUseAprilTagUpdates() {
        return useAprilTagUpdates;
    }
}

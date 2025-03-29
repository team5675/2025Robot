package frc.robot.subsystems.Swerve;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.estimator.*;
import frc.robot.*;
import frc.robot.subsystems.Swerve.TunerConstants.TunerSwerveDrivetrain;

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

    public SwerveDrivePoseEstimator m_poseEstimator;
    public Field2d m_field;

    public double aprilTagCache = -1;

    private String limelightName;

    private double periodicInitTimestamp;
    private double periodicEndTimestamp;

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
        m_poseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(),
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions,
            this.getState().Pose
        );

        this.getPigeon2().reset();

        this.m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        limelightName = Constants.LimelightConstants.lowerLimelightName;

        // LimelightHelpers.setCameraPose_RobotSpace(Constants.LimelightConstants.lowerLimelightName, Constants.LimelightConstants.limelightForward,
        // Constants.LimelightConstants.limelightSide,Constants.LimelightConstants.limelightUp, Constants.LimelightConstants.limelightRoll,
        // Constants.LimelightConstants.limelightPitch, Constants.LimelightConstants.limelightYaw);  // Since your Limelight faces forward, CAMERA_YAW should be 0°
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
        m_poseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(),
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions,
            this.getState().Pose
        );

        this.m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        limelightName = Constants.LimelightConstants.lowerLimelightName;
       
        this.getPigeon2().reset();
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
        m_poseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(),
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions,
            this.getState().Pose
        );

        this.m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        limelightName = Constants.LimelightConstants.lowerLimelightName;
       
       this.getPigeon2().reset();
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

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        periodicInitTimestamp = Timer.getFPGATimestamp();

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

        double redAllianceYaw = this.getPigeon2().getYaw().getValueAsDouble();

        // If on Red Alliance add 180° to the yaw
        if(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
            redAllianceYaw += 180;
        }
        //Pose Estimation using AprilTags
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            redAllianceYaw,
            0, 0, 0, 0, 0
        );
        LimelightHelpers.PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        //Cache the AprilTag ID
        if(LimelightHelpers.getTV(limelightName) && limelightEstimate != null && limelightEstimate.tagCount > 0) {
            aprilTagCache = LimelightHelpers.getFiducialID(limelightName);
        }

        if (limelightEstimate != null && limelightEstimate.tagCount > 0) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 0.7));
            m_poseEstimator.addVisionMeasurement(limelightEstimate.pose, limelightEstimate.timestampSeconds);
        }

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        periodicEndTimestamp = Timer.getFPGATimestamp();

        //Debug Values
        SmartDashboard.putNumber("Robot Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Yaw", this.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("CacheID", aprilTagCache);
        SmartDashboard.putNumber("Robot X", this.m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Robot Y", this.m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Periodic ExecutionTime", periodicEndTimestamp - periodicInitTimestamp);
        
    }

    public void configAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> m_poseEstimator.getEstimatedPosition(),   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController( 
                    // PID constants for translation. Move the P value up until you hear oscillation (in my experience it’s pretty obvious). Then turn it down until there’s no oscillation. 
                    //Usually good values land around 3-5 for the translation components iirc. Sometimes you’ll want a D value but I haven’t seen its value. 
                    //Hopefully the PID won’t be doing much work if your robot is able to follow the path.
                    new PIDConstants(4,0,0), /* Rotation controller:kP = 5.0;kI = 0;kD = 0.0; Position (x/y) controllers:kP = 2.5;kI = 0;kD = 0.0; */
                    //Try kP: 3.0, kI: 0.0, kD: 0.0
                    // PID constants for rotation
                    //Try kP: 5.0, kI: 0.0, kD: 0.0
                    new PIDConstants(3,0,0) //Team 1466’s tuning for rotation was P = 5.0, I = 0.03, D = 0.4
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

        xVelocity = MathUtil.applyDeadband(xVelocity, 0.05);
        yVelocity = MathUtil.applyDeadband(yVelocity, 0.05);
        // angularVelocity = MathUtil.applyDeadband(angularVelocity, 0.05);
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

    public void toggleLimelightSource() {
    if (limelightName.equals(Constants.LimelightConstants.lowerLimelightName)) {
        limelightName = Constants.LimelightConstants.upperLimelightName;
    } else {
        limelightName = Constants.LimelightConstants.lowerLimelightName;
    }

    System.out.println("Switched Limelight to: " + limelightName);
    }
}
package frc.robot.subsystems.Elevator;

public final class ElevatorConstants {

	public static final int motorID = 15;
	public static final int bottomLimitSwitchChannel = 1;
	public static final int topLimitSwitchChannel = 2;

    public static final double motorP = 0.1;
    public static final double motorI = 0.00;
    public static final double motorD = 0.00;

    // Unit is pid ticks
    public static final double L4_HEIGHT = 93;
	public static final double L3_HEIGHT = 48;
	public static final double L2_HEIGHT = 19;
	public static final double L1_HEIGHT = 4;
	public static final double IDLE_HEIGHT = 0;
	public static final double ALGAE_LOW_HEIGHT = 48;
	public static final double ALGAE_HIGH_HEIGHT = 70;

	///public static final double MAX_HEIGHT = 3.0;
}
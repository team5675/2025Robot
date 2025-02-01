package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class ElevatorConstants {
    public static final double motorP = 0.02;
    public static final double motorI = 0.00;
    public static final double motorD = 0.00;

    public static final Distance L4_HEIGHT = Meters.of(2.5146);
	public static final Distance L3_HEIGHT = Meters.of(1.54305);
	public static final Distance L2_HEIGHT = Meters.of(1.54305);
	public static final Distance L1_HEIGHT = Meters.of(1);
	public static final Distance IDLE_HEIGHT = Meters.of(0.1);
	public static final Distance ALGAE_LOW_HEIGHT = Meters.of(1);
	public static final Distance ALGAE_HIGH_HEIGHT = Meters.of(1);
	public static final Distance ALGAE_PROCESSOR_HEIGHT = Meters.of(1);

	public static final Distance MAX_HEIGHT = Meters.of(3.0);

	public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(0.5);
	
	public static final double PID_TICKS_PER_METER = 5.0;

    public static final LinearVelocity ZEROING_VELOCITY = MetersPerSecond.of(0.25);
}

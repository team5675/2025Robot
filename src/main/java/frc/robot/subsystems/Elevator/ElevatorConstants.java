package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;

public final class ElevatorConstants {

	public static final int motorID = 27;
	public static final int bottomLimitSwitchChannel = 1;
	// public static final int topLimitSwitchChannel = 2;

	// slot 0
	public static final double upMotorP = 0.112; //0.112
	public static final double upMotorI = 0.00;
	public static final double upMotorD = 0.00;
	public static final double upMotorff = 0.00;

	public static final ClosedLoopSlot upPidSlot = ClosedLoopSlot.kSlot0;
	
	// slot 1
	public static final double downMotorP = 0.112;
	public static final double downMotorI = 0.00;
	public static final double downMotorD = 0.00;
	public static final double downMotorff = 0.00;

	public static final ClosedLoopSlot downPidSlot = ClosedLoopSlot.kSlot1;

}

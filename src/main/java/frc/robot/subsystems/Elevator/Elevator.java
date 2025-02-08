// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Elevator;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.MetersPerSecond;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.LimitSwitchConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// public class Elevator extends SubsystemBase {
//   private static Elevator instance;

//   public static Elevator getInstance() {
//     if (instance == null) {
//       instance = new Elevator();
//     }

//     return instance;
//   }

//   public SparkMax motor;
//   public SparkMaxConfig motorConfig;
//   private SparkClosedLoopController sparkPidController;
//   private DigitalInput limitSwitch;

//   SparkAbsoluteEncoder angleEncoder;
//   RelativeEncoder ticksEncoder;

//   public Elevator() {
//     motor = new SparkMax(10, MotorType.kBrushless);
//     motorConfig = new SparkMaxConfig();

//     sparkPidController = motor.getClosedLoopController();
//     //pidController = new ProfiledPIDController(ElevatorConstants.motorP, ElevatorConstants.motorI, ElevatorConstants.motorD, null);
    
//     angleEncoder = motor.getAbsoluteEncoder();
//     ticksEncoder = motor.getEncoder();

//     motorConfig.smartCurrentLimit(0);
//     motorConfig.voltageCompensation(12);
//     motorConfig.idleMode(IdleMode.kBrake);

//     motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

//     limitSwitch = new DigitalInput(0);
//   }

// 	public void runElevator() {
// 		this.setTarget(ElevatorConstants.L4_HEIGHT);
// 	}

// 	public void setTarget(double ticks) {
//     sparkPidController.setReference(ticks, ControlType.kPosition);
//     SmartDashboard.putNumber("Ticks", ticks);
//     SmartDashboard.putNumber("Process Variable", ticksEncoder.getPosition());
// 	}

// 	public void zero() {
// 		double zeroingSpeed = -ElevatorConstants.ZEROING_VELOCITY.in(MetersPerSecond);

// 		if (!limitSwitch.get()) {
//       zeroingSpeed = 0;
//     }

//     // add detection based on voltage spike
// 		sparkPidController.setReference(zeroingSpeed, ControlType.kVelocity);
// 	}

//   @Override
//   public void periodic() {
//     //SmartDashboard.putNumber(, voltage)
//   }
// }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
  public SparkMax motor;
  public SparkMaxConfig motorConfig;
  private SparkClosedLoopController sparkPidController;

  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private Trigger bottomTrigger;
  private Trigger topTrigger;

  //SparkAbsoluteEncoder angleEncoder;
  public RelativeEncoder ticksEncoder;

  private final TrapezoidProfile.Constraints trapezoidConstraints = new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCEL);
  private final ProfiledPIDController profilePID = new ProfiledPIDController(ElevatorConstants.profileP, ElevatorConstants.profileI, ElevatorConstants.profileD, trapezoidConstraints, ElevatorConstants.profileDt);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.profileS, ElevatorConstants.profileG, ElevatorConstants.profileV);

  public Elevator() {
    motor = new SparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    sparkPidController = motor.getClosedLoopController();
    // pidController = new ProfiledPIDController(ElevatorConstants.motorP,
    // ElevatorConstants.motorI, ElevatorConstants.motorD, null);

    //angleEncoder = motor.getAbsoluteEncoder();
    ticksEncoder = motor.getEncoder();

    motorConfig.smartCurrentLimit(30, 35);
    motorConfig.voltageCompensation(12);
    motorConfig.idleMode(IdleMode.kBrake);

    // temp replacement for trapezoidprofile
    motorConfig.closedLoopRampRate(0.5);
    
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.motorP, ElevatorConstants.motorI, ElevatorConstants.motorD);

    var status = motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchChannel);
    topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchChannel);

    bottomTrigger = new Trigger(bottomLimitSwitch::get);
    topTrigger = new Trigger(topLimitSwitch::get);
  }

  public void setTarget(double ticks) {
    SmartDashboard.putNumber("Ticks in subsystem", ticks);
    sparkPidController.setReference(ticks, ControlType.kPosition);

    //profilePID.setGoal(ticks);
  }

  public void reset() {
    sparkPidController.setReference(0, ControlType.kPosition);

    //profilePID.setGoal(0);
  }

  @Override
  public void periodic() {
    // flip for some reason
    var bottomBool = !bottomTrigger.getAsBoolean();
    var topBool = topTrigger.getAsBoolean();
    
    if (!bottomBool) {
      // motor.set(0);
      ticksEncoder.setPosition(0);
    }

    //  (!topBool) {
    //   motor.set(0);
    // }

    // set the motor
    //motor.setVoltage(profilePID.calculate(ticksEncoder.getPosition()) + feedforward.calculate(profilePID.getSetpoint().velocity));
 
    SmartDashboard.putBoolean("Top Tripped", topBool);
    SmartDashboard.putBoolean("Bottom Tripped", bottomBool);
    SmartDashboard.putNumber("Elevator Current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Process Variable", ticksEncoder.getPosition());
  }

  private static Elevator instance;
  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

}
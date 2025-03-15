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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.ElevatorLevel;

public class Elevator extends SubsystemBase {
  public SparkMax motor;
  public SparkMaxConfig motorConfig;
  private SparkClosedLoopController sparkPidController;

  private DigitalInput bottomLimitSwitch;
  // private DigitalInput topLimitSwitch;
  public Trigger bottomTrigger;
  // public Trigger topTrigger;

  // SparkAbsoluteEncoder angleEncoder;
  public RelativeEncoder ticksEncoder;

  public ElevatorLevel setPoint = ElevatorLevel.RESET_HEIGHT;

  public Elevator() {
    motor = new SparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    sparkPidController = motor.getClosedLoopController();

    ticksEncoder = motor.getEncoder();

    motorConfig.smartCurrentLimit(30, 35)
        .voltageCompensation(12)
        .idleMode(IdleMode.kBrake)
        .closedLoopRampRate(0.15);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(ElevatorConstants.motorP, ElevatorConstants.motorI, ElevatorConstants.motorD, ElevatorConstants.motorff);
    // motorConfig.closedLoop.maxMotion
    // .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
    // .maxAcceleration(5000.0)
    // .maxVelocity(4000.0)
    // .allowedClosedLoopError(.1);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchChannel);
    // topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchChannel);

    bottomTrigger = new Trigger(bottomLimitSwitch::get);
    // topTrigger = new Trigger(topLimitSwitch::get);
  }

  public void setTarget(ElevatorLevel level) {
    setPoint = level;
    sparkPidController.setReference(level.getLevel(), ControlType.kPosition);
  }

  public void reset() {
    sparkPidController.setReference(0, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // flip - so false = tripped
    var bottomBool = bottomTrigger.getAsBoolean();
    // var topBool = topTrigger.getAsBoolean();

    // If we are resetting and the limit switch is hit
    // if (!bottomBool && (setPoint == ElevatorLevel.RESET_HEIGHT)) {
    //   // motor.set(0);
    //   ticksEncoder.setPosition(0);
    // }
    if (!bottomBool) { 
      ticksEncoder.setPosition(0); // Reset encoder
    if (setPoint.getLevel() == ElevatorLevel.RESET_HEIGHT.getLevel()) {
        motor.set(0); // Ensure motor stops at bottom
    }
}

    // SmartDashboard.putBoolean("Elevator: Top Tripped", topBool);
    SmartDashboard.putBoolean("Elevator: Bottom Tripped", bottomBool);
    SmartDashboard.putNumber("Elevator: Current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator: Motor Output", motor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator: Ticks", ticksEncoder.getPosition());
    SmartDashboard.putNumber("Elevator: Target Position", setPoint.getLevel());
    SmartDashboard.putNumber("Elevator: Position Error", setPoint.getLevel() - ticksEncoder.getPosition());
    SmartDashboard.putNumber("Elevator: Motor Velocity", ticksEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Setpoint", setPoint.getLevel());
  }

  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }
}
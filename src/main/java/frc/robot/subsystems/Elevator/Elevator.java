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
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.LED.LEDStateManager;

public class Elevator extends SubsystemBase {
  public SparkMax motor;
  public SparkMaxConfig motorConfig;
  public MAXMotionConfig maxMotionConfig;
  private SparkClosedLoopController sparkPidController;

  private DigitalInput bottomLimitSwitch;
  public Trigger bottomTrigger;
  public RelativeEncoder ticksEncoder;

  public ElevatorLevel setPoint = ElevatorLevel.RESET_HEIGHT;

  private boolean hasReset = false;

  public Elevator() {
    motor = new SparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    maxMotionConfig = new MAXMotionConfig();

    sparkPidController = motor.getClosedLoopController();

    ticksEncoder = motor.getEncoder();

    motorConfig.smartCurrentLimit(30, 35)
        .voltageCompensation(12)
        .idleMode(IdleMode.kBrake)
        .closedLoopRampRate(0.15);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ElevatorConstants.motorP).i(ElevatorConstants.motorI).d(ElevatorConstants.motorD).velocityFF(0).maxOutput(1).minOutput(-1)
        .maxMotion.maxVelocity(5000.0)
        .maxAcceleration(5000.0)
        .allowedClosedLoopError(.5);
    
        
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchChannel);
    // topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchChannel);

    bottomTrigger = new Trigger(bottomLimitSwitch::get);
    // topTrigger = new Trigger(topLimitSwitch::get);
  }

  public void setTarget(ElevatorLevel level) {
    setPoint = level;
    sparkPidController.setReference(level.getLevel(), ControlType.kMAXMotionPositionControl);
  }

  public void reset() {
    sparkPidController.setReference(0, ControlType.kPosition);
  }

  boolean bottomBool;

  @Override
  public void periodic() {
    bottomBool = bottomTrigger.getAsBoolean();
    // flip - so false = tripped
    
    // var topBool = topTrigger.getAsBoolean();

    // If we are resetting and the limit switch is hit
    // if (!bottomBool && (setPoint == ElevatorLevel.RESET_HEIGHT)) {
    //   // motor.set(0);
    //   ticksEncoder.setPosition(0);
    // }
    if (!bottomBool && setPoint.getLevel() == ElevatorLevel.RESET_HEIGHT.getLevel() && !hasReset) { 
      ticksEncoder.setPosition(0); // Reset encoder
      motor.set(0); //Stop motor for safety
      hasReset = true;
    }
    else if (bottomBool) {
        hasReset = false; // Reset flag when limit switch is un-tripped
    }

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
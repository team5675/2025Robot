// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Coral extends SubsystemBase {
  
  public SparkMax motor;
  SparkMaxConfig motorConfig;
  public DigitalInput beamBreak1;
  public DigitalInput beamBreak2;
  public Trigger bb1Tripped;
  public Trigger bb2Tripped;
  
  public Coral() {
    motor = new SparkMax(CoralConstants.motorID, MotorType.kBrushless);
    
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(CoralConstants.voltsStallLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    beamBreak1 = new DigitalInput(CoralConstants.bb1Channel);
    beamBreak2 = new DigitalInput(CoralConstants.bb2Channel);
    
    bb1Tripped = new Trigger(beamBreak1::get);
    bb2Tripped = new Trigger(beamBreak2::get);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.getBoolean("1", beamBreak1.get());
    SmartDashboard.getBoolean("2", beamBreak2.get());
  }
  
  public static Command PlaceCommand() {
    return Commands.runOnce(() -> {
      Coral.getInstance().motor.set(1);
    });
  }
  
  public static Coral instance;
  public static Coral getInstance() {
    if (instance == null) {
      instance = new Coral();
    }
    return instance;
  }
}

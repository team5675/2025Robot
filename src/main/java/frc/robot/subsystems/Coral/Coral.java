// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.hardware.CANdi;
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
  public CANdi bbCANdi;
  public boolean bb1Tripped;
  public boolean bb2Tripped;
  public static Coral instance;
  public static boolean intaking;

  public Coral() {
    motor = new SparkMax(CoralConstants.motorID, MotorType.kBrushless);

    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(CoralConstants.stallLimit);
    motorConfig.idleMode(IdleMode.kBrake);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    bbCANdi = new CANdi(CoralConstants.bbCANdi_ID);

    Coral.intaking = false;
  }

  @Override
  public void periodic() {
    bb1Tripped = bbCANdi.getS1Closed().getValue();
    bb2Tripped = bbCANdi.getS2Closed().getValue();
    SmartDashboard.putBoolean("Beam Break 1", bb1Tripped);
    SmartDashboard.putBoolean("Beam Break 2", bb2Tripped);
    SmartDashboard.putBoolean("Intaking", intaking);
  }

  public static Command PlaceCommand() {
    return Commands.runOnce(() -> {
      Coral.getInstance().motor.set(1);
    });
  }

  // Check with Connor to see if these are the right values
  public static Boolean isTripped() {
    return Coral.getInstance().bb1Tripped && Coral.getInstance().bb2Tripped;
  }

  public static Boolean clearToMove() {
    return !intaking;
  }


  public static Coral getInstance() {
    if (instance == null) {
      instance = new Coral();
    }
    return instance;
  }
}
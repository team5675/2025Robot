// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
  
  private SparkMax wheelsMotor; //spins the algae intake wheels
  private SparkMax axisMotor; //rotates the algae mechanism out and in

  private SparkClosedLoopController axisPID;

  private RelativeEncoder axisTicks;

  private SparkMaxConfig wheelsConfig;
  private SparkMaxConfig axisConfig;

  public Algae() {
    // wheel motor configs
    wheelsMotor = new SparkMax(AlgaeConstants.wheelsID, MotorType.kBrushless);
    
    wheelsConfig = new SparkMaxConfig();
    wheelsConfig.smartCurrentLimit(AlgaeConstants.voltsStallLimit);
    wheelsConfig.idleMode(IdleMode.kBrake);

    wheelsMotor.configure(wheelsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    //axis motor config
    axisMotor = new SparkMax(AlgaeConstants.axisID, MotorType.kBrushless);
    axisTicks = axisMotor.getEncoder();
    
    axisConfig = new SparkMaxConfig();
    axisConfig.idleMode(IdleMode.kBrake);
    axisConfig.smartCurrentLimit(AlgaeConstants.voltsStallLimit);
    
    axisPID = axisMotor.getClosedLoopController();
    axisConfig.closedLoop
    .pid(AlgaeConstants.axisP, AlgaeConstants.axisI, AlgaeConstants.axisD)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    axisMotor.configure(axisConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
  @Override
  public void periodic() {
    
    //If the motor is stalled (> 15 Amps) then stop the motor
    //and reset encoder due to hard stop
    if (axisMotor.getOutputCurrent() > 15) {
      axisMotor.set(0);
      
      if (Math.abs(axisTicks.getPosition()) < AlgaeConstants.axisTicksTolerance) { // within 5 ticks of zero
        axisTicks.setPosition(0);
      }
    }
    
    SmartDashboard.putNumber("Axis Ticks", axisTicks.getPosition());
    SmartDashboard.putNumber("Axis Current", axisMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel Current", wheelsMotor.getOutputCurrent());
  }
  
  //moves the axis out
  public void AxisOut() {
    System.out.println("Out");
    axisPID.setReference(AlgaeConstants.AxisOutTicks, ControlType.kPosition);
  }
  
  //brings axis back in
  public void AxisIn() {
    System.out.println("in");
    axisPID.setReference(AlgaeConstants.AxisInTicks, ControlType.kPosition);
  }
  
  //spins the wheels based on input speed
  public void flywheelSpin(double speed) {
    System.out.println("Spinning Wheel!");
    wheelsMotor.set(speed);
  }
  
  private static Algae instance;
  public static Algae getInstance() {
    if (instance == null) {
      instance = new Algae();
    }
    return instance;
  }
}

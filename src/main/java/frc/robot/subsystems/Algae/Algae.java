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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Algae extends SubsystemBase {
  
  public SparkMax wheelsMotor; //spins the algae intake wheels
  public SparkMax axisMotor; //rotates the algae mechanism out and in

  private SparkClosedLoopController axisPID;

  public RelativeEncoder axisTicks;

  private SparkMaxConfig wheelsConfig;
  private SparkMaxConfig axisConfig;

  private Trigger axisSpike;

  public static boolean intaking;

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
    
    axisTicks.setPosition(0);

    axisSpike = new Trigger(this::spike);

    setAxisPosition(5);
    axisTicks.setPosition(0);
  }
  
  @Override
  public void periodic() {
    //If the motor is stalled (> 15 Amps) then stop the motor
    //and reset encoder due to hard stop
    // if (axisSpike.getAsBoolean()) {
    //   axisMotor.set(0);
      
    //   if (Math.abs(axisTicks.getPosition()) < AlgaeConstants.axisTicksTolerance) { // within 5 ticks of zero
    //     axisTicks.setPosition(0);
    //   }
    // }

    
    if(!intaking && axisSpike.getAsBoolean()){
      axisMotor.set(0);
      setAxisPosition(0);
    }
    else if(!intaking && axisTicks.getPosition() > -0.5){
      axisMotor.set(0);
      axisTicks.setPosition(0);
    }
    else if(!intaking && axisTicks.getPosition() < -0.5 && !axisSpike.getAsBoolean()){
      axisMotor.set(.25);
      
    }
    // if(axisSpike.getAsBoolean() && axisTicks.getPosition() < -60){
    //   setAxisPosition(0);
    //   axisTicks.setPosition(0);
    // }
    // else if(axisSpike.getAsBoolean() && axisTicks.getPosition() > -20){
    //   setAxisPosition(0);
    //   axisTicks.setPosition(0);
      
    // }
    
    SmartDashboard.putNumber("Axis Ticks", axisTicks.getPosition());
    SmartDashboard.putNumber("Axis Current", axisMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel Current", wheelsMotor.getOutputCurrent());
  }
  
  public void setAxisPosition(double position) {
    // negative because the ticks are flipped
    axisPID.setReference(-position, ControlType.kPosition);
  }
  
  //spins the wheels based on input speed
  public void setFlywheelSpeed(double speed) {
    System.out.println("Spinning Wheel!");
    wheelsMotor.set(speed);
  }

  public Boolean spike() {
    return axisMotor.getOutputCurrent() > 10;
  }

  public void setIntake(boolean bool){
    intaking = bool;
  }
  
  private static Algae instance;
  public static Algae getInstance() {
    if (instance == null) {
      instance = new Algae();
    }
    return instance;
  }
}

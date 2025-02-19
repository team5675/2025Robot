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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Algae extends SubsystemBase {
  private static Algae instance;
  public static Algae getInstance() {
    if (instance == null) {
      instance = new Algae();
    }
    return instance;
  }

  private SparkMax wheelsMotor;
  private SparkMax axisMotor;
  private SparkClosedLoopController axisPID;
  private RelativeEncoder axisTicks;
  //private DigitalInput axisHardStop;
  //Trigger axisHardStopTripped;

  public Algae() {
    wheelsMotor = new SparkMax(AlgaeConstants.wheelsID, MotorType.kBrushless);

    SparkMaxConfig wheelsConfig = new SparkMaxConfig();
    wheelsConfig.smartCurrentLimit(15);

    wheelsMotor.configure(wheelsConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    axisMotor = new SparkMax(AlgaeConstants.axisID, MotorType.kBrushless);
    axisTicks = axisMotor.getEncoder();

    SparkMaxConfig axisConfig = new SparkMaxConfig();
    axisConfig.idleMode(IdleMode.kBrake);
    axisConfig.smartCurrentLimit(15);

    axisPID = axisMotor.getClosedLoopController();
    axisConfig.closedLoop
        .pid(AlgaeConstants.axisP, AlgaeConstants.axisI, AlgaeConstants.axisD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    axisMotor.configure(axisConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // axisHardStop = new DigitalInput(1);
    // axisHardStopTripped = new Trigger(axisHardStop::get);
  }

  public void AxisOut() {
    System.out.println("Out");
    axisPID.setReference(200, ControlType.kPosition);
  }

  public void AxisIn() {
    System.out.println("in");
    axisPID.setReference(0, ControlType.kPosition);
  }

  public void flywheelSpin(double speed) {
    System.out.println("Spinning Wheel!");
    wheelsMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    // if (axisHardStopTripped.getAsBoolean()) {
    //   ticksEncoder.setPosition(0);
    // }

    if (axisMotor.getOutputCurrent() > 15) {
      axisMotor.set(0);

      if (Math.abs(axisTicks.getPosition()) < 5) { // within 5 ticks of zero
        axisTicks.setPosition(0);
      }
    }

    SmartDashboard.putNumber("Axis Ticks", axisTicks.getPosition());
    SmartDashboard.putNumber("Axis Current", axisMotor.getOutputCurrent());

    SmartDashboard.putNumber("Flywheel Current", wheelsMotor.getOutputCurrent());
  }
}

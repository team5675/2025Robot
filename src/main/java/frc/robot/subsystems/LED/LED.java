// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static LED instance;

  public static LED getInstance() {
    if (instance == null) {
      instance = new LED();
    }
    return instance;
  }

  public AddressableLED led;
  public AddressableLEDBuffer ledBuffer;

  public LED() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(90);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }
  
  @Override
  public void periodic() {

  }
}

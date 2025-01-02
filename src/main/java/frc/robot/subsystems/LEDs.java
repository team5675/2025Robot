// Copyright (c) 2023 FRC 5675
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class LEDs extends SubsystemBase {

  public static LEDs instance;
  private Spark blinkinLED; //Maybe Static??

  public void createBlinkin() {
    blinkinLED = new Spark(1);
  }

  public void setRainbow() {
    blinkinLED.set(Constants.LEDConstants.LEDRainbow); //Rainbow
  }

  public void setOrange() {
    blinkinLED.set(Constants.LEDConstants.LEDOrange); //Orange
  }

  public void turnOff() {
    blinkinLED.set(0); 
  }

  public void BlueDim() {
    blinkinLED.set(0.01);
  }

  public void RedDim() {
    blinkinLED.set(0.21); 
  }


  public void setAllianceColor() {

    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Blue){
        blinkinLED.set(Constants.LEDConstants.LEDBlue); //Blue
        } else {
          blinkinLED.set(Constants.LEDConstants.LEDRed); //0.61 for normal red
        }
      }
  }
  public static LEDs getInstance() {
    if (instance == null) 
        instance = new LEDs();

    return instance;
}
}

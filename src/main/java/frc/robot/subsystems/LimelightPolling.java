// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.commands.LimelightCommand;

public class LimelightPolling extends SubsystemBase {
  /** Creates a new Limelight. */
  public ArrayList<LimelightCommand> tables;
  public LimelightPolling() {
    tables = new ArrayList<LimelightCommand>();
    // add new limelights here
    tables.add(new LimelightCommand(Constants.LimelightConstants.limelightName));

    for (LimelightCommand ll : tables) {
      CommandScheduler.getInstance().schedule(ll);
    }
  }

  private static LimelightPolling instance;
  public static LimelightPolling getInstance() {
    if (instance == null) {
      instance = new LimelightPolling();
    } 
    return instance;
  }
  
  @Override
  public void periodic() {

  }
}
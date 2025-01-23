// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightCommand extends Command {
  public NetworkTable limelightTable; 
  public String limelightName;
  public boolean limelightEnabled;

  public LimelightCommand(String limelightName) {
    this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    this.limelightName = limelightName;
    this.limelightEnabled = true;
  }

  public double tx;
  public double ty;
  public double tid;

  @Override
  public void execute() {
    this.tx = this.limelightTable.getEntry("tx").getDouble(-1);
    this.ty = this.limelightTable.getEntry("ty").getDouble(-1);
    this.tid = this.limelightTable.getEntry("tid").getDouble(-1);
  }
}
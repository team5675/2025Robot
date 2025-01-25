// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightCommand;
import frc.robot.subsystems.limelight.LimelightPolling;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathPlannerLineup extends Command {
  Command pathfindingCommand;
  LimelightCommand limelight;
  final CommandSwerveDrivetrain drivetrain;
  final String direction;
  Pose2d pose;
  
  /** Creates a new PathPlannerLineup. */
  public PathPlannerLineup(CommandSwerveDrivetrain drivetrain, String direction) {
    this.drivetrain = drivetrain;
    this.direction = direction;
    limelight = LimelightPolling.getInstance().limelights.get(Constants.LimelightConstants.limelightName);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      if (direction == "left") {
       switch ((int)limelight.tid) {
        case 3: pose = Constants.AlignmentConstants.A_BLUE;
        case 4: pose = Constants.AlignmentConstants.A_BLUE;
        case 5: pose = Constants.AlignmentConstants.A_BLUE;
        case 6: pose = Constants.AlignmentConstants.A_BLUE;
        case 7: pose = Constants.AlignmentConstants.A_BLUE;
        case 8: pose = Constants.AlignmentConstants.A_BLUE;
        case 9: pose = Constants.AlignmentConstants.A_BLUE;
        case 10: pose = Constants.AlignmentConstants.A_BLUE;
        case 13: pose = Constants.AlignmentConstants.A_BLUE;
        case 14: pose = Constants.AlignmentConstants.A_BLUE;
        case 16: pose = Constants.AlignmentConstants.A_BLUE;
        case 17: pose = Constants.AlignmentConstants.A_BLUE;
        case 18: pose = Constants.AlignmentConstants.A_BLUE;
       }

    } else { // right
      switch ((int)limelight.tid) {
        case 3: pose = Constants.AlignmentConstants.A_BLUE;
        case 4: pose = Constants.AlignmentConstants.A_BLUE;
        case 5: pose = Constants.AlignmentConstants.A_BLUE;
        case 6: pose = Constants.AlignmentConstants.A_BLUE;
        case 7: pose = Constants.AlignmentConstants.A_BLUE;
        case 8: pose = Constants.AlignmentConstants.A_BLUE;
        case 9: pose = Constants.AlignmentConstants.A_BLUE;
        case 10: pose = Constants.AlignmentConstants.A_BLUE;
        case 13: pose = Constants.AlignmentConstants.A_BLUE;
        case 14: pose = Constants.AlignmentConstants.A_BLUE;
        case 16: pose = Constants.AlignmentConstants.A_BLUE;
        case 17: pose = Constants.AlignmentConstants.A_BLUE;
        case 18: pose = Constants.AlignmentConstants.B_BLUE;
    }
    }
    pathfindingCommand = AutoBuilder.pathfindToPose(
       pose,
        Constants.PathplannerConstants.constraints,
        0.0 // Goal end velocity in meters/sec
        );

        pathfindingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

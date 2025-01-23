// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveToPoseCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final Pose2d targetPose;

    public MoveToPoseCommand(CommandSwerveDrivetrain swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.moveToPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        return swerve.getState().Pose.getTranslation().getDistance(targetPose.getTranslation()) < 0.05; // 5 cm tolerance
    }
}

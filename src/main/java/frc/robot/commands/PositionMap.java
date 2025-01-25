package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum AlignmentConstants {
    OFF("OFF, No Aligning!", new Pose2d()),
	A("Driving to Reef A", new Pose2d(1.235,7.045, new Rotation2d(12))),
	B("Driving to Reef B", new Pose2d(1.0,1.0, new Rotation2d(12))),
	C("Driving to Reef C", new Pose2d(1.0,1.0, new Rotation2d(12))),
	D("Driving to Reef D", new Pose2d(1.0,1.0, new Rotation2d(12))),
	E("Driving to Reef E", new Pose2d(1.0,1.0, new Rotation2d(12))),
	F("Driving to Reef F", new Pose2d(1.0,1.0, new Rotation2d(12))),
	G("Driving to Reef G", new Pose2d(1.0,1.0, new Rotation2d(12))),
	H("Driving to Reef H", new Pose2d(1.0,1.0, new Rotation2d(12))),
	I("Driving to Reef I", new Pose2d(1.0,1.0, new Rotation2d(12))),
	J("Driving to Reef J", new Pose2d(1.0,1.0, new Rotation2d(12))),
	K("Driving to Reef K", new Pose2d(1.0,1.0, new Rotation2d(12))),
	L("Driving to Reef L", new Pose2d(13.8,2.5, new Rotation2d(147.2))),
	coral1("Driving to coral station 1", new Pose2d()),
	coral3("Driving to coral station 3", new Pose2d()),
    redBarge("Driving to redBarge", new Pose2d()),
    blueBarge("Driving to blueBarge", new Pose2d()),
    processor("Driving to processor", new Pose2d());

    AlignmentConstants(String info, Pose2d pose) {
        this.name = info;
        this.pose = pose;
    }

    String name;
    Pose2d pose;

    public String getPoseName() {
		return name;
	}

	public Pose2d getTargetPose() {
		return pose;
	}
}

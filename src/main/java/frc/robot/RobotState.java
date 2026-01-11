package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
	private static RobotState instance;

	// Singleton pattern
	public static RobotState getInstance() {
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}

	// TODO: add observation functions for vision and drive odometry

	public Pose2d getEstimatedRobotPose() {
		return Pose2d.kZero;
	}
}
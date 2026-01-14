package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;

public class RobotState {
  private static RobotState instance;

  // Singleton pattern
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private Pose2d odometryPose = Pose2d.kZero;

  public void addOdometryObservation(OdometryObservation observation) {
    final var robotDisplacement =
        Arrays.stream(observation.moduleDisplacements)
            .reduce(Translation2d.kZero, Translation2d::plus)
            .rotateBy(observation.gyroYaw);
    final Pose2d poseEstimate =
        new Pose2d(odometryPose.getTranslation().plus(robotDisplacement), observation.gyroYaw);

    odometryPose = poseEstimate;
  }

  // TODO: add observation functions for vision and drive odometry
  public Pose2d getEstimatedRobotPose() {
    return Pose2d.kZero;
  }

  public record OdometryObservation(
      double timestamp, Rotation2d gyroYaw, Translation2d[] moduleDisplacements) {}
}

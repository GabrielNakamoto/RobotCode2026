package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.StateSubsystem;

public class Vision extends StateSubsystem<frc.robot.subsystems.vision.Vision.SystemState> {
  public enum SystemState {
    ALL_TAGS,
  }

  private VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] inputs;

  public Vision(VisionIO[] cameras) {
    this.cameras = cameras;
    this.inputs = new VisionIOInputsAutoLogged[cameras.length];
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(inputs[i]);
    }
  }

  @Override
  protected void applyState() {
    switch (getCurrentState()) {
      case ALL_TAGS:
        for (int i = 0; i < cameras.length; ++i) {
          final VisionIOInputsAutoLogged input = inputs[i];
          final Pose2d estimate = poseEstimate2d(input.tx, input.ty, input.tid, i);
          RobotState.getInstance()
              .addVisionObservation(new AprilTagObservation(input.timestamp, estimate));
        }
        break;
    }
  }

  /*
   * Estimates robot pose relative to april tag using only 2 DOF (yaw and distance)
   * Simpler and more robust to noise during motion than 6 DOF methods like Megatag
   */
  public Pose2d poseEstimate2d(double tx, double ty, int tid, int index) {
    final Pose3d tagPose = FieldConstants.aprilLayout.getTagPose(tid).get();
    final Transform3d robotToCamera = VisionConstants.robotToCameras[index];
    final Rotation3d camRot = robotToCamera.getRotation();

    // ground distance = tag height / tan(camera pitch + ty)
    final double distance = tagPose.getZ() / Math.tan(camRot.getY() + Math.toRadians(ty));

    // Project camera tx to ground tx: tan(ground tx) = tan(cam tx) / cos(cam yaw)
    final double groundTx = Math.atan(Math.tan(Math.toRadians(tx)) / Math.cos(camRot.getZ()));

    final Transform2d tagToCamera =
        new Transform2d(
            new Translation2d(distance, Rotation2d.fromRadians(-groundTx)), Rotation2d.kZero);

    final Pose2d cameraPose = tagPose.toPose2d().transformBy(tagToCamera);
    return cameraPose.transformBy(VisionConstants.cameraToRobot2d[index]);
  }
}

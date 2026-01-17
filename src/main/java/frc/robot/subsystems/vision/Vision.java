package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.StateSubsystem;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Vision extends StateSubsystem<frc.robot.subsystems.vision.Vision.SystemState> {
  public enum SystemState {
    SINGLE_TAG_2D,
  }

  private VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] inputs;
  private final Consumer<Angle> setYaw;

  public Vision(VisionIO[] cameras, Consumer<Angle> setYaw) {
    this.setYaw = setYaw;
    this.cameras = cameras;
    this.inputs = new VisionIOInputsAutoLogged[cameras.length];
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/camera" + i, inputs[i]);
    }

    applyState();
  }

  @Override
  protected void applyState() {
    switch (getCurrentState()) {
      case SINGLE_TAG_2D:
        for (int i = 0; i < cameras.length; ++i) {
          final VisionIOInputsAutoLogged input = inputs[i];
          final Pose2d estimate = poseEstimate2d(input.tx, input.ty, input.tid, i);

          if (input.update_yaw) setYaw.accept(input.megatagYaw);
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

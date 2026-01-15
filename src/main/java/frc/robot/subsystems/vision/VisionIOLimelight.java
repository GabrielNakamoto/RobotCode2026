package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {
  // Apriltag positions
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;

  private final IntegerSubscriber tidSubscriber;

  // Latency
  private final DoubleSubscriber tlSubscriber;
  private final DoubleSubscriber clSubscriber;

  private final DoubleArraySubscriber megatagSubscriber;
  private final int index;

  public VisionIOLimelight(int index) {
    this.index = index;

    var table = NetworkTableInstance.getDefault().getTable(VisionConstants.cameraNames[index]);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    tidSubscriber = table.getIntegerTopic("tid").subscribe(0);
    tlSubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    clSubscriber = table.getDoubleTopic("cl").subscribe(0.0);

    megatagSubscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.tx = txSubscriber.get();
    inputs.ty = tySubscriber.get();
    inputs.tid = (int) tidSubscriber.get();

    final double latency = (tlSubscriber.get() + clSubscriber.get()) / 1e3;
    inputs.timestamp = Timer.getFPGATimestamp() - latency;

    // Reset gyro yaw with megatag estimate
    // TODO: filter this so we dont reset every pass
    inputs.megatagYaw = Degrees.of(megatagSubscriber.get()[5]);
    inputs.update_yaw = false;
  }
}

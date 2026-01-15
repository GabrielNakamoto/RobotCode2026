package frc.robot.subsystems.vision;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    double tx;
    double ty;
    int tid;
    double timestamp;
    Angle megatagYaw;
    boolean update_yaw;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}

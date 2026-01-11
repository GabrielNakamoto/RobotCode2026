package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        // Do we need pitch or roll?
        public boolean isConnected = false;
        public Rotation2d yaw = Rotation2d.kZero;
        public AngularVelocity yawVelocity = RadiansPerSecond.of(0);
    }

    public void updateInputs(GyroIOInputs inputs);
}

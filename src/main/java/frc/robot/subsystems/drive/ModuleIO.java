package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public Distance drivePosition = Meters.of(0.0);
    public LinearVelocity driveVelocity = MetersPerSecond.of(0.0);

    public boolean turnConnected = false;
    public Rotation2d absoluteTurnHeading = Rotation2d.kZero;
    public AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public static class ModuleIOOutputs {
    public LinearVelocity driveVelocity;
    public Rotation2d turnHeading;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void applyOutputs(ModuleIOOutputs outputs) {}
}

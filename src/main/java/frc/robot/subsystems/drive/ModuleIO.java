package frc.robot.subsystems.drive;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ModuleIO {
  @AutoLog
	public static class ModuleIOInputs {
		public boolean driveConnected = false;
		public Distance drivePosition = Meters.of(0.0);
		public AngularVelocity driveVelocity = RadiansPerSecond.of(0.0);

		public boolean turnConnected = false;
		public Rotation2d absoluteTurnHeading = Rotation2d.kZero;
		public AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);
	}

	public static class ModuleIOOutputs {
		public LinearVelocity driveVelocity;
		public Rotation2d turnHeading;
	}

	public void updateInputs(ModuleIOInputs inputs);
	public void applyOutputs(ModuleIOOutputs outputs);
}

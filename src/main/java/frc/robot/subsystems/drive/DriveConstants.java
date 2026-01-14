package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class DriveConstants {
  public static final double maxLinearSpeed = 0.0;
  public static final Distance wheelRadius = Meters.of(0.0);
  public static final boolean motionMagicSteerControl = false;

  public static final Translation2d[] modulePositions = {
    new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d(),
  };
}

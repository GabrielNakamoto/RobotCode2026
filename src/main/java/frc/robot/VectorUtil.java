package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VectorUtil {
  public static Translation2d tangentUnitVector(Translation2d v) {
    return v.rotateBy(Rotation2d.fromDegrees(90)).div(v.getNorm());
  }
}

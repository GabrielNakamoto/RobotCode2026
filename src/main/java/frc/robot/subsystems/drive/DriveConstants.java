package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class DriveConstants {
  public static record PIDGains(double kp, double ki, double kd) {}
  ;

  public static final TrapezoidProfile.Constraints rotConstraints =
      new TrapezoidProfile.Constraints(0.0, 0.0);
  public static final PIDGains driveGains = new PIDGains(0.0, 0.0, 0.0);
  public static final PIDGains rotGains = new PIDGains(0.0, 0.0, 0.0);
  public static final double maxLinearSpeed = 6.7;
  public static final double maxRotationalSpeed = 12.0;
  public static final Distance wheelRadius = Inches.of(2.6);
  public static final boolean motionMagicSteerControl = false;
  public static final Distance driveTolerance = Meters.of(0.0);
  public static final Angle rotateTolerance = Degrees.of(0.0);

  private static final double trackWidthMeters = Constants.config.trackWidth().in(Meters);
  private static final double trackLengthMeters = Constants.config.trackLength().in(Meters);
  public static final Translation2d[] modulePositions = {
    new Translation2d(trackLengthMeters / 2, -trackWidthMeters / 2),
    new Translation2d(trackLengthMeters / 2, trackWidthMeters / 2),
    new Translation2d(-trackLengthMeters / 2, -trackWidthMeters / 2),
    new Translation2d(-trackLengthMeters / 2, trackWidthMeters / 2),
  };

  // Feed forward
  public static final double drivekS = 5.0;
  public static final double drivekV = 0.0;

  // Sim specific
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Constants.config.mass())
          .withBumperSize(Constants.config.bumperLength(), Constants.config.bumperWidth())
          .withCustomModuleTranslations(modulePositions)
          .withGyro(COTS.ofPigeon2());
}

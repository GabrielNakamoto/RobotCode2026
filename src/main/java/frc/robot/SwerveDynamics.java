package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Module;
import org.ejml.simple.SimpleMatrix;

/*
 * Custom swerve drive kinematics and odometry math implementation
 *
 * Advantages over WPIlib are primarily for greater understanding,
 * and customizability
 *
 * Inspired by Orbit 1690
 */
public class SwerveDynamics {
  public static class ChassisVelocity {
    public AngularVelocity omega = RadiansPerSecond.zero();
    public Translation2d velocityVector = Translation2d.kZero;
    // Precompute pseudo inverse chasiss matrix to reduce per cycle allocations
    public static final SimpleMatrix pseudoMatrix =
        new SimpleMatrix(
                new double[][] {
                  {1, 0, -DriveConstants.modulePositions[0].getY()},
                  {0, 1, DriveConstants.modulePositions[0].getX()},
                  {1, 0, -DriveConstants.modulePositions[1].getY()},
                  {0, 1, DriveConstants.modulePositions[1].getX()},
                  {1, 0, -DriveConstants.modulePositions[2].getY()},
                  {0, 1, DriveConstants.modulePositions[2].getX()},
                  {1, 0, -DriveConstants.modulePositions[3].getY()},
                  {0, 1, DriveConstants.modulePositions[3].getX()},
                })
            .pseudoInverse();

    public ChassisVelocity() {}

    public ChassisVelocity(AngularVelocity omega, Translation2d v) {
      this.omega = omega;
      this.velocityVector = v;
    }

    // Forward kinematics
    // Uses least squares method to solve unknown vector
    // https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Linear_least-squares
    public static ChassisVelocity forwardKinematics(Module[] modules) {
      SimpleMatrix B = new SimpleMatrix(8, 1);

      for (int i = 0; i < 4; ++i) {
        final Translation2d vel = modules[i].getVelocity();

        B.set(i * 2, vel.getX());
        B.set((i * 2) + 1, vel.getY());
      }

      // x ~= A+ @ b
      final var bestFit = pseudoMatrix.mult(B);
      return new ChassisVelocity(
          RadiansPerSecond.of(bestFit.get(2)), new Translation2d(bestFit.get(0), bestFit.get(1)));
    }

    public ModuleVelocity[] inverseKinematics(Module[] modules) {
      var mvs = new ModuleVelocity[4];

      for (int i = 0; i < 4; ++i) {
        final Translation2d chassisPos = modules[i].getChassisPosition();
        final double radiusMeters = chassisPos.getNorm();
        final double rotationalVelMps = omega.in(RadiansPerSecond) * radiusMeters;
        Translation2d rotationVector =
            VectorUtil.tangentUnitVector(chassisPos).times(rotationalVelMps);
        mvs[i] = new ModuleVelocity(this.velocityVector.plus(rotationVector));
      }

      return mvs;
    }
  }

  public static class ModuleVelocity {
    private Translation2d velocity;

    public ModuleVelocity(Translation2d inner) {
      this.velocity = inner;
    }

    public ModuleVelocity(double magnitude, Rotation2d angle) {
      this.velocity = new Translation2d(magnitude, angle);
    }

    public ModuleVelocity scale(double scalar) {
      this.velocity = velocity.times(scalar);
      return this;
    }

    public final Rotation2d getHeading() {
      return velocity.getAngle();
    }

    public final LinearVelocity magnitude() {
      return MetersPerSecond.of(velocity.getNorm());
    }

    public final double getSpeedMps() {
      return velocity.getNorm();
    }

    public void optimize(Rotation2d heading) {
      closestAngle(heading);
      cosineOptimize(heading);
    }

    // Reduces speed opposite to rotation vector
    private void cosineOptimize(Rotation2d heading) {
      this.velocity = velocity.times(velocity.getAngle().minus(heading).getCos());
    }

    // Prevents uneccessary heading changes > 90°, takes opposite angle
    private void closestAngle(Rotation2d heading) {
      double angleError = MathUtil.angleModulus(velocity.getAngle().minus(heading).getRadians());
      if (Math.abs(angleError) > Math.PI / 2) {
        this.velocity =
            new Translation2d(-velocity.getNorm(), velocity.getAngle().plus(Rotation2d.k180deg));
      }
    }
  }

  /* Improves odometry accuracy by accounting for circular movement
   * in module displacements
   *
   * returns a module relative arc displacement vectorChief
   *
   * Chief delphi thread: https://www.chiefdelphi.com/t/orbit-1690-2025-robot-reveal-whisper/492064/150?u=raine1
   */
  public static Translation2d getArcDisplacement(
      Translation2d chassisPos,
      Distance lastPos,
      Distance curPos,
      Rotation2d lastHeading,
      Rotation2d curHeading) {
    final Distance linearDist = curPos.minus(lastPos);
    final Rotation2d theta = curHeading.minus(lastHeading);
    final Rotation2d moduleToChassis = lastHeading;

    // Handle small theta values, else radius calculation will
    // blow up, just return linear displacement
    if (Math.abs(theta.getRadians()) < 1e-6) {
      return new Translation2d(linearDist.in(Meters), moduleToChassis);
    }

    // r = a / theta
    final double radius = linearDist.in(Meters) / theta.getRadians();
    final Translation2d centerToLast =
        new Translation2d(radius, lastHeading.minus(Rotation2d.fromDegrees(90)));
    final Translation2d centerToCur = centerToLast.rotateBy(theta);

    return centerToCur.minus(centerToLast);
  }
}

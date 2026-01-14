package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.Drive.RobotVelocity;

import static edu.wpi.first.units.Units.*;

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
		Rotation2d curHeading
	) {
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
		final Translation2d centerToLast = new Translation2d(
			radius,
			lastHeading.minus(Rotation2d.fromDegrees(90))
		);
		final Translation2d centerToCur = centerToLast.rotateBy(theta);

		return centerToCur.minus(centerToLast);
	}

	// Forward kinematics
	// Uses least squares method to solve unknown vector
	// https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Linear_least-squares
	public static RobotVelocity modulesToChassisVelocity(Module[] modules) {
		SimpleMatrix A = new SimpleMatrix(8, 3);
		SimpleMatrix B = new SimpleMatrix(8, 1);

		for (int i=0; i<4; ++i) {
			final Translation2d pos = modules[i].getPosition();
			final Translation2d vel = modules[i].getVelocity();

			A.setRow(i*2, new SimpleMatrix(new double[] {1, 0, -pos.getY()}));
			A.setRow((i*2)+1, new SimpleMatrix(new double[] {0, 1, pos.getX()}));
			B.set(i*2, vel.getX());
			B.set((i*2)+1, vel.getY());
		}

		// x ~= A+ @ b
		final var bestFit = A.pseudoInverse().mult(B);
		return new RobotVelocity(
			RadiansPerSecond.of(bestFit.get(2)),
			new Translation2d(bestFit.get(0), bestFit.get(1))
		);
	}

	// Inverse kinematics to calculate individual Module heading and velocity 
	// based on requested, chassis relative rotational and linear velocities
	public static Translation2d chassisToModuleVelocity(
		Translation2d moduleChassisPosition,
		Translation2d chassisTranslationVelocity,
		AngularVelocity omega
	) {
		final double radiusMeters = moduleChassisPosition.getNorm();
		final double rotationalVelMps = omega.in(RadiansPerSecond) * radiusMeters;

		Translation2d rotationVector = VectorUtil.tangentUnitVector(moduleChassisPosition)
			.times(rotationalVelMps);
		return chassisTranslationVelocity.plus(rotationVector);
	}

	// Reduces speed opposite to rotation vector
	public static Translation2d cosineOptimize(Translation2d velocity, Rotation2d heading) {
		return velocity.times(velocity.getAngle().minus(heading).getCos());
	}

	// Prevents uneccessary heading changes > 90°, takes opposite angle
	public static Translation2d closestAngle(Translation2d velocity, Rotation2d heading) {
		double angleError = MathUtil.angleModulus(velocity.getAngle().minus(heading).getRadians());
		if (angleError > Math.PI / 2) {
			return new Translation2d(
				-velocity.getNorm(),
				velocity.getAngle().plus(Rotation2d.k180deg)
			);
		} else {
			return velocity;
		}
	}
}

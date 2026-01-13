package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputs;

public class Module {
    private final ModuleIO io;
    private final Translation2d chassisPosition;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private ModuleIOOutputs outputs = new ModuleIOOutputs();
    private final int index;

		private Distance lastPos = Meters.of(0.0);
		private Rotation2d lastHeading = Rotation2d.kZero;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        this.chassisPosition = DriveConstants.modulePositions[index];
    }

		// Forward kinematics (chassis velocities -> module velocities)
		// Inspired by 1690 Orbit: https://www.youtube.com/watch?v=vUtVXz7ebEE&t=322s
    public Translation2d chassisToModule(Translation2d translationVelocity, AngularVelocity omega) {
				// w = r x v
        final double radiusMeters = chassisPosition.getNorm();
        final LinearVelocity rotationalVel = MetersPerSecond.of(omega.in(RadiansPerSecond) * radiusMeters);

				// TODO: Positive counter clockwise?
        Translation2d tangentVector = chassisPosition.rotateBy(Rotation2d.fromDegrees(90));
        tangentVector = tangentVector.div(tangentVector.getNorm());
        final Translation2d rotationVelocity = tangentVector.times(rotationalVel.in(MetersPerSecond));

        return translationVelocity.plus(rotationVelocity);
    }

		// Improves odometry accuracy by accounting for circular movement
		// in module displacements
		//
		// Also inspired by 1690 orbit:
		// https://www.chiefdelphi.com/t/orbit-1690-2025-robot-reveal-whisper/492064/150?u=raine1
		public Translation2d getArcDisplacement() {
			// arc length = theta x r
			// r = arc length / theta
			final var arcLength = inputs.drivePosition.minus(lastPos);
			final var theta = inputs.absoluteTurnHeading.minus(lastHeading);
			final var moduleToChassis = lastHeading;

			lastPos = inputs.drivePosition;
			lastHeading = inputs.absoluteTurnHeading;

			// Handle small theta values, radius calculation will blow up
			// just return straight line distance
			if (Math.abs(theta.getRadians()) < 1e-6) {
				return new Translation2d(arcLength.in(Meters), moduleToChassis);
			}

			final double radius = arcLength.in(Meters) / theta.getRadians();
			// Arc vectors are perpindicular to module direction
			final Translation2d centerToPrev = new Translation2d(radius, theta.minus(Rotation2d.fromRadians(Math.PI / 2)));
			final Translation2d centerToCur = centerToPrev.rotateBy(theta);
			final Translation2d arcDisplacement = centerToCur.minus(centerToPrev);

			return arcDisplacement.rotateBy(moduleToChassis);
		}

		// Prioritizes turning by decreasing speed
		// Reduces speed if module isnt pointing in desired direction
		// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
		private Translation2d cosineOptimize(Translation2d velocity) {
			final var angleError = velocity.getAngle()
				.minus(inputs.absoluteTurnHeading);
			final double factor = Math.abs(angleError.getCos());
			return velocity.times(factor);
		}

		// Ensures wheel doesnt rotate > 90°, chooses faster path
		private Translation2d angleOptimize(Translation2d velocity) {
			double targetAngle = velocity.getAngle().getRadians();
			double currentAngle = inputs.absoluteTurnHeading.getRadians();

			// Wraps pi -> -pi
			double angleError = MathUtil.angleModulus(targetAngle - currentAngle);
			if (Math.abs(angleError) > Math.PI / 2) {
				return new Translation2d(
						velocity.getNorm(),
						Rotation2d.fromRadians(targetAngle + Math.PI)
				);
			} else {
				return velocity;
			}
		}

    public void runVelocity(Translation2d velocity) {
				velocity = angleOptimize(velocity);
				velocity = cosineOptimize(velocity);

        outputs.turnHeading = velocity.getAngle();
        outputs.driveVelocity = MetersPerSecond.of(velocity.getNorm()); 
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);

        io.applyOutputs(outputs);
    }
}

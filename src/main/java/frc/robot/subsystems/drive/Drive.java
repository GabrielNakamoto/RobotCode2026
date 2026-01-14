package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Comparator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.StateSubsystem;
import frc.robot.SwerveDynamics;
import frc.robot.VectorUtil;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;


public class Drive extends StateSubsystem<frc.robot.subsystems.drive.Drive.SystemState> {
	public enum SystemState {
		IDLE,
		TELEOP_DRIVE,
		TO_POSE,
	};

	public static class RobotVelocity {
		public AngularVelocity omega = RadiansPerSecond.of(0);
		public Translation2d headingVelocity = Translation2d.kZero;

		public RobotVelocity() {}
		public RobotVelocity(AngularVelocity omega, Translation2d headingVel) {
			this.omega = omega;
			this.headingVelocity = headingVel;	
		}

		public RobotVelocity chassisRelative(Rotation2d yaw) {
			return new RobotVelocity(
				omega,
				this.headingVelocity.rotateBy(yaw)
			);
		}
	};

	private final Module[] swerveModules = new Module[4];
	private final GyroIO gyro;

	private final GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();
	private RobotVelocity fieldRelVelocity = new RobotVelocity();
	private RobotVelocity currentChassisVelocity = new RobotVelocity();
	private Pose2d targetPose = Pose2d.kZero;

	private PIDController linearController = new PIDController(0, 0, 0);
	private ProfiledPIDController omegaController = new ProfiledPIDController(0, 0, 0,
			new TrapezoidProfile.Constraints(0, 0));


	public Drive(
			GyroIO gyro,
			ModuleIO flModuleIO,
			ModuleIO frModuleIO,
			ModuleIO blModuleIO,
			ModuleIO brModuleIO) {
		this.gyro = gyro;
		this.swerveModules[0] = new Module(flModuleIO, 0);
		this.swerveModules[1] = new Module(frModuleIO, 1);
		this.swerveModules[2] = new Module(blModuleIO, 2);
		this.swerveModules[3] = new Module(brModuleIO, 3);

		omegaController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void periodic() {
		gyro.updateInputs(gyroData);
		Logger.processInputs("Drive/Gryo", gyroData);

		for (Module m : swerveModules) {
			m.periodic();
		}

		updateState();
		final Translation2d[] moduleDisplacements = Arrays.stream(swerveModules)
			.map(Module::getDisplacement)
			.toArray(Translation2d[]::new);

		RobotState.getInstance().addOdometryObservation(new OdometryObservation(
					Timer.getFPGATimestamp(),
					gyroData.yaw,
					moduleDisplacements
		));

		// TODO: update chassis velocities
	}

	// Forward kinematics?
	public RobotVelocity getChassisVelocity() {
		Translation2d vl = Translation2d.kZero;
		AngularVelocity omega = RadiansPerSecond.of(0.0);

		for (Module m : swerveModules) {
			final Translation2d mv = m.getVelocity();
			final Translation2d mp = m.getPosition();

			vl = vl.plus(mv);
			final double tangentSpeed = VectorUtil.tangentUnitVector(mp).dot(mv);

			if (mp.getNorm() < 1e-6) continue;
			omega = omega.plus(RadiansPerSecond.of(tangentSpeed / mp.getNorm())); // w = v / r
		}
		return new RobotVelocity(
			omega.div(swerveModules.length),
			vl.div(swerveModules.length)
		);
	}

	@Override
	protected void applyState() {
		switch (getCurrentState()) {
			case TELEOP_DRIVE:
				runChassisRelativeVelocity(fieldRelVelocity.chassisRelative(gyroData.yaw));
				break;
			case TO_POSE: // TODO: implement
				break;
			default: break;
		}
	}

	private void runChassisRelativeVelocity(RobotVelocity chassisVel) {
		final Translation2d translationVel = chassisVel.headingVelocity;
		final AngularVelocity omega = chassisVel.omega;

		var moduleVelocities = Arrays.stream(swerveModules)
			.map(m -> SwerveDynamics.chassisToModuleVelocity(
				m.getPosition(), 
				translationVel,
				omega
			));
		final double maxVelocity = moduleVelocities
			.mapToDouble(v -> v.getNorm())
			.max()
			.orElse(0.0);

		// Normalize module velocities to preserve direction when exceeding speed limits
		// Accounts for max drive + max turn requested at same time
		if (maxVelocity > DriveConstants.maxLinearSpeed) {
			final double factor = DriveConstants.maxLinearSpeed / maxVelocity;
			moduleVelocities = moduleVelocities.map(v -> v.times(factor));
		}

		final var mvs = moduleVelocities.toList();
		for (int i = 0; i < 4; ++	i) {
			swerveModules[i].runVelocity(mvs.get(i));
		}
	}

	public void setTargetPose(Pose2d pose) {
		this.targetPose = pose;

		linearController.reset();
	}

	public void setFieldRelativeVelocity(RobotVelocity velocity) {
		this.fieldRelVelocity = velocity;
	}
}

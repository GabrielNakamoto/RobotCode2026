package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateSubsystem;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

public class Drive extends StateSubsystem<frc.robot.subsystems.drive.Drive.SystemState> {
	public enum SystemState {
		IDLE,
		TELEOP_DRIVE,
		TO_POSE,
	};

	private final Module[] swerveModules = {};
	private final GyroIO gyro;

	private final GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();
	private Translation2d fieldRelVelocity = Translation2d.kZero;

	public Drive(GyroIO gyro) {
		this.gyro = gyro;
	}

	@Override
	public void periodic() {
		gyro.updateInputs(gyroData);
		updateState();
	}

	@Override
	protected void applyState() {
		switch (getCurrentState()) {
		}
	}


	private Translation2d chassisRelativeVelocity() {
		return this.fieldRelVelocity.rotateBy(gyroData.yaw);
	}
	
	public void setFieldRelativeVelocity(Translation2d velocity) {
		this.fieldRelVelocity = velocity;
	}
}

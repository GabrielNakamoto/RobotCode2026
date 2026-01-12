package frc.robot.subsystems.drive;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public abstract class ModuleIOTalonFX implements ModuleIO {
  protected final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;
  protected final TalonFX driveMotor;
  protected final TalonFX turnMotor;
  protected final CANcoder cancoder;

  protected final StatusSignal<Angle> drivePosition;
  protected final StatusSignal<AngularVelocity> driveVelocity;

  protected final StatusSignal<Angle> turnPositionAbsolute;
  protected final StatusSignal<AngularVelocity> turnVelocity;


  public ModuleIOTalonFX(
    SwerveModuleConstants<
      TalonFXConfiguration,
      TalonFXConfiguration,
      CANcoderConfiguration
    > constants
  ) {
    this.constants = constants;
    driveMotor = new TalonFX(constants.DriveMotorId);
    turnMotor = new TalonFX(constants.SteerMotorId);
    cancoder = new CANcoder(constants.EncoderId);

    driveVelocity = driveMotor.getVelocity();
    drivePosition = driveMotor.getPosition();
    turnPositionAbsolute = cancoder.getAbsolutePosition();
    turnVelocity = turnMotor.getVelocity();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = driveMotor.isConnected();
    inputs.driveVelocity = driveVelocity.getValue();
    inputs.drivePosition = drivePosition.getValue();

    inputs.turnConnected = turnMotor.isConnected();
    inputs.absoluteTurnHeading = new Rotation2d(turnPositionAbsolute.getValue());
    inputs.turnVelocity = turnVelocity.getValue();
  }

  @Override
  public void applyOutputs(ModuleIOOutputs outputs) {
		// Velocity control for drive
		// Position control for steer
  }
}

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.PheonixOdometryThread;
import java.util.Queue;

public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
  protected final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Queue<Double> timestamps;
  private final Queue<Double> drivePositions;
  private final Queue<Double> turnPositions;

  public ModuleIOTalonFXReal(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    super(constants);
    this.constants = constants;

    timestamps = PheonixOdometryThread.getInstance().makeTimestampQueue();
    drivePositions = PheonixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    turnPositions =
        PheonixOdometryThread.getInstance().registerSignal(cancoder.getAbsolutePosition());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    inputs.odometryTimestamps = timestamps.stream().mapToDouble(x -> x).toArray();
    inputs.odometryDrivePositionsRad = drivePositions.stream().mapToDouble(x -> x).toArray();
    inputs.odometryTurnPositions =
        (Rotation2d[]) drivePositions.stream().map(x -> Rotation2d.fromRadians(x)).toArray();
  }
}

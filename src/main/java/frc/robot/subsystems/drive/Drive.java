package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.StateSubsystem;
import frc.robot.SwerveDynamics.ChassisVelocity;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateSubsystem<frc.robot.subsystems.drive.Drive.SystemState> {
  public enum SystemState {
    IDLE,
    TELEOP_DRIVE,
    TO_POSE,
  };

  private final Module[] swerveModules = new Module[4];
  private final GyroIO gyro;

  private final GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();

  private Translation2d fieldVelocity = Translation2d.kZero;
  private AngularVelocity fieldOmega = RadiansPerSecond.zero();

  private ChassisVelocity currentChassisVelocity = new ChassisVelocity();
  private Pose2d targetPose = Pose2d.kZero;

  private PIDController linearController = new PIDController(0, 0, 0);
  private ProfiledPIDController omegaController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private Translation2d[] moduleDisplacements = new Translation2d[4];

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

    currentChassisVelocity = ChassisVelocity.forwardKinematics(swerveModules);
    moduleDisplacements =
        Arrays.stream(swerveModules).map(Module::getDisplacement).toArray(Translation2d[]::new);

    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(Timer.getFPGATimestamp(), gyroData.yaw, moduleDisplacements));

    updateState();
  }

  @Override
  protected void applyState() {
    switch (getCurrentState()) {
      case TELEOP_DRIVE:
        runChassisRelativeVelocity(
            new ChassisVelocity(fieldOmega, fieldVelocity.rotateBy(gyroData.yaw)));
        break;
      case TO_POSE:
        break;
      default:
        break;
    }
  }

  public void setYaw(Angle yaw) {
    gyro.setYaw(yaw);
  }

  private ChassisVelocity getCurrentChassisVelocity() {
    return ChassisVelocity.forwardKinematics(swerveModules);
  }

  private void runChassisRelativeVelocity(ChassisVelocity velocity) {
    var moduleVelocities = velocity.inverseKinematics(swerveModules);
    final double maxVelocity =
        Arrays.stream(moduleVelocities).mapToDouble(v -> v.getSpeedMps()).max().orElse(0.0);

    // Normalize module velocities to preserve direction when exceeding speed limits
    // Accounts for max drive + max turn requested at same time
    if (maxVelocity > DriveConstants.maxLinearSpeed) {
      final double factor = DriveConstants.maxLinearSpeed / maxVelocity;
      Arrays.stream(moduleVelocities).map(v -> v.scale(factor));
    }

    for (int i = 0; i < 4; ++i) {
      swerveModules[i].runVelocity(moduleVelocities[i]);
    }
  }

  public void setTargetPose(Pose2d pose) {
    this.targetPose = pose;

    linearController.reset();
  }

  public void setFieldRelativeVelocity(Translation2d velocity, AngularVelocity omega) {
    this.fieldVelocity = velocity;
    this.fieldOmega = omega;
  }
}

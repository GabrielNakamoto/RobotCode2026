package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.StateSubsystem;
import frc.robot.SwerveDynamics.ChassisVelocity;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputMode;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateSubsystem<frc.robot.subsystems.drive.Drive.SystemState> {
  public enum SystemState {
    IDLE,
    TELEOP_DRIVE,
    TO_POSE_PID,
    TEST
  };

  private final Module[] swerveModules = new Module[4];
  private final GyroIO gyro;

  private final GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();

  private PIDController linearController =
      new PIDController(
          DriveConstants.driveGains.kp(),
          DriveConstants.driveGains.ki(),
          DriveConstants.driveGains.kd());
  private ProfiledPIDController omegaController =
      new ProfiledPIDController(
          DriveConstants.rotGains.kp(),
          DriveConstants.rotGains.ki(),
          DriveConstants.rotGains.kd(),
          DriveConstants.rotConstraints);
  private Translation2d[] moduleDisplacements = new Translation2d[4];
  private XboxController controller;

  private Pose2d driveToPointPose = new Pose2d();

  public Drive(
      GyroIO gyro,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      XboxController controller) {
    this.controller = controller;
    this.gyro = gyro;
    this.swerveModules[0] = new Module(flModuleIO, 0);
    this.swerveModules[1] = new Module(frModuleIO, 1);
    this.swerveModules[2] = new Module(blModuleIO, 2);
    this.swerveModules[3] = new Module(brModuleIO, 3);

    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    linearController.setTolerance(DriveConstants.driveTolerance.in(Meters));
    omegaController.setTolerance(DriveConstants.rotateTolerance.in(Radians));

    // setState(SystemState.TEST);
    setState(SystemState.TO_POSE_PID);
    driveToPointPose = new Pose2d(2, 3, Rotation2d.kZero);
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroData);
    Logger.processInputs("Drive/Gryo", gyroData);
    Logger.recordOutput("Drive/systemState", getCurrentState());

    for (var m : swerveModules) {
      m.periodic();
    }

    statePeriodic();

    if (DriverStation.isDisabled()) {
      for (var m : swerveModules) {
        m.setMode(ModuleIOOutputMode.BRAKE);
      }
    }

    /*
      RobotState.getInstance()
          .addOdometryObservation(
              new OdometryObservation(Timer.getFPGATimestamp(), gyroData.yaw, moduleDisplacements));
    */

    for (var m : swerveModules) {
      m.periodicAfter();
    }
  }

  @Override
  protected void applyState() {
    switch (getCurrentState()) {
      case TELEOP_DRIVE:
        runChassisRelativeVelocity(getControllerSpeeds());
        break;
      case TO_POSE_PID:
        pidToPose();
        break;
      case TEST:
        runChassisRelativeVelocity(
            new ChassisVelocity(RadiansPerSecond.zero(), new Translation2d(0.5, 0.0)));
        break;
      default:
        break;
    }
  }

  @Override
  protected SystemState handleStateTransitions() {
    var requested = getRequestedState();
    switch (requested) {
      case TO_POSE_PID:
        linearController.reset();
        omegaController.reset(
            gyroData.yaw.getRadians(), getCurrentChassisVelocity().omega.in(RadiansPerSecond));
        break;
      default:
        break;
    }
    return requested;
  }

  public void setYaw(Angle yaw) {
    gyro.setYaw(yaw);
  }

  private void pidToPose() {
    // var robotPose = RobotState.getInstance().getEstimatedRobotPose();
    var robotPose = RobotState.getInstance().getSimulatedPose();
    var robotToTarget = driveToPointPose.getTranslation().minus(robotPose.getTranslation());
    var distance = robotToTarget.getNorm();
    var heading = robotToTarget.getAngle();

    var output = Math.min(linearController.calculate(distance, 0), DriveConstants.maxLinearSpeed);
    var vx = output * heading.getCos();
    var vy = output * heading.getSin();

    var vw =
        Math.min(
            omegaController.calculate(
                robotPose.getRotation().getRadians(), driveToPointPose.getRotation().getRadians()),
            DriveConstants.maxRotationalSpeed);

    if (linearController.atSetpoint() && omegaController.atSetpoint()) {
      setState(SystemState.TELEOP_DRIVE);
      runChassisRelativeVelocity(new ChassisVelocity());
    } else {
      var robotVelocity = new ChassisVelocity(RadiansPerSecond.of(vw), new Translation2d(vx, vy));
      Logger.recordOutput("PidToPose/outputOmega", vw);
      Logger.recordOutput("PidToPose/outputLinear", new Translation2d(vx, vy));
      runChassisRelativeVelocity(robotVelocity);
    }
  }

  private static final double CONTROLLER_DEADBAND = 0.1;

  private ChassisVelocity getControllerSpeeds() {
    final double sX = MathUtil.applyDeadband(controller.getLeftY(), CONTROLLER_DEADBAND);
    final double sY = MathUtil.applyDeadband(controller.getLeftX(), CONTROLLER_DEADBAND);
    double sW = MathUtil.applyDeadband(controller.getRightX(), CONTROLLER_DEADBAND);

    sW = Math.copySign(sW * sW, sW); // heuristic?
    final double sign = FieldConstants.isBlueAlliance() ? -1 : 1;
    final double vX = sX * DriveConstants.maxLinearSpeed * sign;
    final double vY = sY * DriveConstants.maxLinearSpeed * sign;
    final double vW = sW * DriveConstants.maxRotationalSpeed;

    return ChassisVelocity.fromFieldRelative(vX, vY, vW, gyroData.yaw);
  }

  public final ChassisVelocity getCurrentChassisVelocity() {
    return ChassisVelocity.forwardKinematics(swerveModules);
  }

  private void runChassisRelativeVelocity(ChassisVelocity velocity) {
    var moduleVelocities = velocity.inverseKinematics(swerveModules);
    for (int i = 0; i < 4; ++i) {
      swerveModules[i].runVelocity(moduleVelocities[i]);
    }
  }
}

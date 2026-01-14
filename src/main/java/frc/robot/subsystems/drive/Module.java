package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.SwerveDynamics;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputs;
import org.littletonrobotics.junction.Logger;

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

  public Translation2d getPosition() {
    return chassisPosition;
  }

  public Translation2d getDisplacement() {
    final Translation2d displacement =
        SwerveDynamics.getArcDisplacement(
            chassisPosition,
            lastPos,
            inputs.drivePosition,
            lastHeading,
            inputs.absoluteTurnHeading);
    lastPos = inputs.drivePosition;
    lastHeading = inputs.absoluteTurnHeading;
    return displacement;
  }

  public Translation2d getVelocity() {
    return new Translation2d(inputs.driveVelocity.in(MetersPerSecond), inputs.absoluteTurnHeading);
  }

  public void runVelocity(Translation2d velocity) {
    velocity = SwerveDynamics.closestAngle(velocity, inputs.absoluteTurnHeading);
    velocity = SwerveDynamics.cosineOptimize(velocity, inputs.absoluteTurnHeading);

    outputs.turnHeading = velocity.getAngle();
    outputs.driveVelocity = MetersPerSecond.of(velocity.getNorm());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    io.applyOutputs(outputs);
  }
}

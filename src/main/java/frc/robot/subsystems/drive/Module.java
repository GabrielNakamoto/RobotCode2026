package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputs;

public class Module {
    private final ModuleIO io;
    private final Translation2d chassisPosition;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private ModuleIOOutputs outputs = new ModuleIOOutputs();
    private final int index;

    public Module(ModuleIO io, Translation2d chassisPosition, int index) {
        this.io = io;
        this.index = index;
        this.chassisPosition = chassisPosition;
    }

    public Translation2d chassisToModule(Translation2d translationVelocity, AngularVelocity omega) {
        // Forward kinematics (chassis velocities -> module velocities)
        final double radiusMeters = chassisPosition.getNorm();
        final LinearVelocity rotationalVel = MetersPerSecond.of(omega.in(RadiansPerSecond) * radiusMeters);
        Translation2d tangentVector =  chassisPosition.rotateBy(Rotation2d.fromDegrees(90));
        tangentVector.div(tangentVector.getNorm());
        final Translation2d rotationVelocity = tangentVector.times(rotationalVel.in(MetersPerSecond));

        return translationVelocity.plus(rotationVelocity);
    }

    public void runVelocity(Translation2d velocity) {
        outputs.turnHeading = velocity.getAngle();
        outputs.driveVelocity = MetersPerSecond.of(velocity.getNorm()); 
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);

        io.applyOutputs(outputs);
    }
}

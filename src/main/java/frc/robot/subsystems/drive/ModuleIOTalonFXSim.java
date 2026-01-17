package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

class TalonFXSimController implements SimulatedMotorController {
  private final TalonFXSimState simState;

  public TalonFXSimController(TalonFX talon) {
    this.simState = talon.getSimState();
  }

  @Override
  public Voltage updateControlSignal(
      Angle mechanismAngle,
      AngularVelocity mechanismVelocity,
      Angle encoderAngle,
      AngularVelocity encoderVelocity) {
    simState.setRotorVelocity(encoderVelocity);
    simState.setRawRotorPosition(encoderAngle);
    simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    return simState.getMotorVoltageMeasure();
  }
}

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation moduleSim;

  public ModuleIOTalonFXSim(
      SwerveModuleSimulation moduleSim,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    super(constants);

    this.moduleSim = moduleSim;
    moduleSim.useDriveMotorController(new TalonFXSimController(driveTalon));
    moduleSim.useSteerMotorController(new TalonFXSimController(turnTalon));
  }
}

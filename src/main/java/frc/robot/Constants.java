package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }
}

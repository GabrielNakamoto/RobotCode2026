package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.io.IOException;
import java.nio.file.Path;

public class FieldConstants {
  public static final AprilTagFieldLayout aprilLayout;

  static {
    try {
      aprilLayout =
          new AprilTagFieldLayout(
              Path.of("src", "main", "deploy", "apriltags", "2026-welded.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}

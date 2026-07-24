// Copyright (c) FRC 5431
//
// Publishes Limelight MJPEG video streams to NetworkTables so dashboards
// (Elastic, Shuffleboard) can display them as Camera Stream widgets.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import java.util.ArrayList;
import java.util.List;

/**
 * Publishes Limelight camera streams under the standard {@code /CameraPublisher/<name>/streams}
 * topic that Elastic and Shuffleboard read to populate Camera Stream widgets.
 *
 * <p>The Limelight serves its MJPEG feed on port 5800. We publish both the mDNS hostname
 * ({@code <name>.local}) and bare-hostname forms as alternates so the dashboard can fall back if
 * one does not resolve on the driver station network.
 */
public final class LimelightCameraStream {
  // Retain publishers so their topics stay published for the life of the robot program.
  // (A garbage-collected publisher would unpublish the topic and drop the widget.)
  private static final List<StringArrayPublisher> publishers = new ArrayList<>();

  private LimelightCameraStream() {}

  /**
   * Publishes the MJPEG stream for a single Limelight.
   *
   * @param limelightName The configured Limelight name (e.g. "limelight-yogev").
   */
  public static void publish(String limelightName) {
    StringArrayPublisher streams =
        NetworkTableInstance.getDefault()
            .getTable("CameraPublisher")
            .getSubTable(limelightName)
            .getStringArrayTopic("streams")
            .publish();
    streams.set(
        new String[] {
          "mjpg:http://" + limelightName + ".local:5800",
          "mjpg:http://" + limelightName + ":5800"
        });
    publishers.add(streams);
  }
}

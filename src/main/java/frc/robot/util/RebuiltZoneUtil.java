package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;

public final class RebuiltZoneUtil {
  private static final double blueEndXMax =
      (FieldConstants.Tower.blueTower.getX() + FieldConstants.Hub.blueCenter.getX()) / 2.0;
  private static final double centerXMax =
      (FieldConstants.Hub.blueCenter.getX() + FieldConstants.Hub.redCenter.getX()) / 2.0;
  private static final double redEndXMin =
      (FieldConstants.Hub.redCenter.getX() + FieldConstants.Tower.redTower.getX()) / 2.0;

  private static final double lowerLaneYMax =
      (FieldConstants.Trench.blueRun.getY() + FieldConstants.Hub.blueCenter.getY()) / 2.0;
  private static final double upperLaneYMin =
      (FieldConstants.Hub.blueCenter.getY() + FieldConstants.Trench.redRun.getY()) / 2.0;

  private RebuiltZoneUtil() {}

  public enum RebuiltZone {
    BLUE_OUTPOST_NEAR(0),
    BLUE_TOWER(1),
    BLUE_OUTPOST_FAR(2),
    BLUE_TRENCH_NEAR(3),
    CENTER_HUB(4),
    BLUE_TRENCH_FAR(5),
    RED_TRENCH_NEAR(6),
    RED_HUB(7),
    RED_TRENCH_FAR(8),
    RED_OUTPOST_NEAR(9),
    RED_TOWER(10),
    RED_OUTPOST_FAR(11),
    UNKNOWN(12);

    private final int id;

    RebuiltZone(int id) {
      this.id = id;
    }

    public int getId() {
      return id;
    }
  }

  public static RebuiltZone getZone(Pose2d pose) {
    double x = clamp(pose.getX(), 0.0, FieldConstants.fieldLength);
    double y = clamp(pose.getY(), 0.0, FieldConstants.fieldWidth);

    if (x <= blueEndXMax) {
      return getBlueEndZone(y);
    }
    if (x < centerXMax) {
      return getBlueMidfieldZone(y);
    }
    if (x < redEndXMin) {
      return getRedMidfieldZone(y);
    }
    if (x <= FieldConstants.fieldLength) {
      return getRedEndZone(y);
    }
    return RebuiltZone.UNKNOWN;
  }

  public static double[] getZoneBoundariesMeters() {
    return new double[] {blueEndXMax, centerXMax, redEndXMin, lowerLaneYMax, upperLaneYMin};
  }

  public static String getZoneName(Pose2d pose) {
    return getZone(pose).name();
  }

  private static RebuiltZone getBlueEndZone(double y) {
    if (y < lowerLaneYMax) {
      return RebuiltZone.BLUE_OUTPOST_NEAR;
    }
    if (y > upperLaneYMin) {
      return RebuiltZone.BLUE_OUTPOST_FAR;
    }
    return RebuiltZone.BLUE_TOWER;
  }

  private static RebuiltZone getBlueMidfieldZone(double y) {
    if (y < lowerLaneYMax) {
      return RebuiltZone.BLUE_TRENCH_NEAR;
    }
    if (y > upperLaneYMin) {
      return RebuiltZone.BLUE_TRENCH_FAR;
    }
    return RebuiltZone.CENTER_HUB;
  }

  private static RebuiltZone getRedMidfieldZone(double y) {
    if (y < lowerLaneYMax) {
      return RebuiltZone.RED_TRENCH_NEAR;
    }
    if (y > upperLaneYMin) {
      return RebuiltZone.RED_TRENCH_FAR;
    }
    return RebuiltZone.RED_HUB;
  }

  private static RebuiltZone getRedEndZone(double y) {
    if (y < lowerLaneYMax) {
      return RebuiltZone.RED_OUTPOST_NEAR;
    }
    if (y > upperLaneYMin) {
      return RebuiltZone.RED_OUTPOST_FAR;
    }
    return RebuiltZone.RED_TOWER;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}

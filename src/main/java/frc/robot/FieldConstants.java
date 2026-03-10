// Copyright (c) 2026 Team 7587 Metuchen Momentum
// https://github.com/frc-team7587
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a Blue Alliance origin. * Field dimensions derived from the 2026 REBUILT Game Manual and
 * Field Dimension Drawings.
 */
public class FieldConstants {
  // Field Dimensions (Welded Perimeter)
  public static final double fieldLength = Units.inchesToMeters(651.22);
  public static final double fieldWidth = Units.inchesToMeters(317.69);

  public static final double tapeLineWidth = Units.inchesToMeters(2.0);

  // Game Piece (Fuel)
  public static final double fuelDiameter = Units.inchesToMeters(5.91);
  public static final double fuelWeightLbs = 0.5; // Upper bound approx

  // -------------------------------------------------------------------------
  // FIELD ELEMENTS
  // -------------------------------------------------------------------------

  public static class Hub {
    public static final Pose2d blueHubFrontScore =
        new Pose2d(
            Units.inchesToMeters(146.11),
            Units.inchesToMeters(158.84),
            Rotation2d.fromDegrees(180));
    public static final Pose2d blueHubBackScore =
        new Pose2d(
            Units.inchesToMeters(218.11), Units.inchesToMeters(158.84), Rotation2d.fromDegrees(0));

    public static final Pose2d redHubFrontScore =
        new Pose2d(
            Units.inchesToMeters(505.11), Units.inchesToMeters(158.84), Rotation2d.fromDegrees(0));
    public static final Pose2d redHubBackScore =
        new Pose2d(
            Units.inchesToMeters(433.11),
            Units.inchesToMeters(158.84),
            Rotation2d.fromDegrees(180));

    public static final double widthFlatToFlat = Units.inchesToMeters(58.41);
    public static final double widthPointToPoint = Units.inchesToMeters(72.00);

    // Manual Section 5.4
    public static final double height = Units.inchesToMeters(120.36);
    public static final double openingHeight = Units.inchesToMeters(72.00);
    public static final double openingWidth = Units.inchesToMeters(41.7);

    // AprilTag mounting height
    public static final double aprilTagHeight = Units.inchesToMeters(44.25);

    // Calculated Center Poses
    public static final Pose2d blueCenter =
        new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d(0));

    public static final Pose2d redCenter =
        new Pose2d(
            Units.inchesToMeters(469.11), Units.inchesToMeters(158.84), new Rotation2d(Math.PI));

    // AprilTag Poses
    public static final Pose2d[] AprilTagPoses = {
      // blue
      new Pose2d(
          Units.inchesToMeters(483.11),
          Units.inchesToMeters(135.09),
          new Rotation2d(Math.toRadians(270))),
      new Pose2d(
          Units.inchesToMeters(492.88),
          Units.inchesToMeters(144.84),
          new Rotation2d(Math.toRadians(0))),
      new Pose2d(
          Units.inchesToMeters(492.88),
          Units.inchesToMeters(158.84),
          new Rotation2d(Math.toRadians(0))),
      new Pose2d(
          Units.inchesToMeters(483.11),
          Units.inchesToMeters(182.60),
          new Rotation2d(Math.toRadians(90))),

      // red
      new Pose2d(
          Units.inchesToMeters(182.11),
          Units.inchesToMeters(135.09),
          new Rotation2d(Math.toRadians(270))),
      new Pose2d(
          Units.inchesToMeters(205.87),
          Units.inchesToMeters(144.84),
          new Rotation2d(Math.toRadians(0))),
      new Pose2d(
          Units.inchesToMeters(205.87),
          Units.inchesToMeters(158.84),
          new Rotation2d(Math.toRadians(0))),
      new Pose2d(
          Units.inchesToMeters(182.11),
          Units.inchesToMeters(182.60),
          new Rotation2d(Math.toRadians(90)))
    };
  }

  public static class Trench {
    // Manual Section 5.6
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);

    // Clearance under the arm
    public static final double clearanceWidth = Units.inchesToMeters(50.34);
    public static final double clearanceHeight = Units.inchesToMeters(22.25);

    // AprilTag mounting height
    public static final double aprilTagHeight = Units.inchesToMeters(35.00);

    // Reference Points (Centers of the runs under the arm)
    public static final Pose2d blueRun =
        new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(25.37), new Rotation2d(0));

    public static final Pose2d redRun =
        new Pose2d(
            Units.inchesToMeters(469.11), Units.inchesToMeters(292.31), new Rotation2d(Math.PI));
  }

  public static class Outpost {
    // Manual Section 5.9.2
    public static final double depth = Units.inchesToMeters(27.0); // Including Depot barrier depth

    public static final double upperOpeningHeight = Units.inchesToMeters(28.1);
    public static final double upperOpeningWidth = Units.inchesToMeters(31.8);

    public static final double lowerOpeningHeight = Units.inchesToMeters(1.88);
    public static final double lowerOpeningWidth = Units.inchesToMeters(32.0);

    // AprilTag mounting height
    public static final double aprilTagHeight = Units.inchesToMeters(21.75);

    // Intake Poses (Centered between the two slots)
    public static final Pose2d blueIntake =
        new Pose2d(
            Units.inchesToMeters(0.30), Units.inchesToMeters(34.72), Rotation2d.fromDegrees(0));

    public static final Pose2d redIntake =
        new Pose2d(
            Units.inchesToMeters(650.92),
            Units.inchesToMeters(282.97),
            Rotation2d.fromDegrees(180));
  }

  public static class Tower {
    // Manual Section 5.8
    public static final double rungSpacing = Units.inchesToMeters(18.0);
    public static final double uprightSpacing = Units.inchesToMeters(32.25);

    // Rung Heights (from floor to center of rung)
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Rung Dimensions
    public static final double rungDiameter = Units.inchesToMeters(1.66); // 1-1/4" Schedule 40
    public static final double rungLength = Units.inchesToMeters(44.0); // 32.25 + 2*5.875

    // AprilTag mounting height
    public static final double aprilTagHeight = Units.inchesToMeters(21.75);

    // Tower Locations (Centered between uprights)
    public static final Pose2d blueTower =
        new Pose2d(
            Units.inchesToMeters(0.32), Units.inchesToMeters(155.97), Rotation2d.fromDegrees(0));

    public static final Pose2d redTower =
        new Pose2d(
            Units.inchesToMeters(650.90),
            Units.inchesToMeters(161.72),
            Rotation2d.fromDegrees(180));
  }

  public static class Depot {
    // Manual Section 5.7
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double barrierHeight = Units.inchesToMeters(1.125);
  }

  public static class Bump {
    // Manual Section 5.5
    public static final double width = Units.inchesToMeters(73.0);
    public static final double depth = Units.inchesToMeters(44.4);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double rampAngle = 15.0; // Degrees
  }

  public static class StartingPoses {
    // --- STARTING POSES ---
    public static final Pose2d blueStartTrenchSide =
        new Pose2d(
            Units.inchesToMeters(30.0), Units.inchesToMeters(25.37), Rotation2d.fromDegrees(0));
    public static final Pose2d blueStartCenter =
        new Pose2d(
            Units.inchesToMeters(30.0), Units.inchesToMeters(158.84), Rotation2d.fromDegrees(0));
    public static final Pose2d blueStartBumpSide =
        new Pose2d(
            Units.inchesToMeters(30.0), Units.inchesToMeters(292.31), Rotation2d.fromDegrees(0));

    public static final Pose2d redStartTrenchSide =
        new Pose2d(
            Units.inchesToMeters(621.22),
            Units.inchesToMeters(292.31),
            Rotation2d.fromDegrees(180));
    public static final Pose2d redStartCenter =
        new Pose2d(
            Units.inchesToMeters(621.22),
            Units.inchesToMeters(158.84),
            Rotation2d.fromDegrees(180));
    public static final Pose2d redStartBumpSide =
        new Pose2d(
            Units.inchesToMeters(621.22), Units.inchesToMeters(25.37), Rotation2d.fromDegrees(180));
  }

  // -------------------------------------------------------------------------
  // APRILTAGS
  // -------------------------------------------------------------------------

  public static final double aprilTagWidth = Units.inchesToMeters(8.125); // Manual 5.11
  public static final int aprilTagCount = 32;

  public static final AprilTagFieldLayout aprilTags =
      new AprilTagFieldLayout(
          List.of(
              // --- RED ALLIANCE SIDE (IDs 1-16) ---

              // 1: Trench, Red (Far side from Scoring Table)
              new AprilTag(
                  1,
                  new Pose3d(
                      Units.inchesToMeters(467.64),
                      Units.inchesToMeters(292.31),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(180)))),

              // 2-5: Red Hub (Ring 1)
              new AprilTag(
                  2,
                  new Pose3d(
                      Units.inchesToMeters(469.11),
                      Units.inchesToMeters(182.60),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(90)))),
              new AprilTag(
                  3,
                  new Pose3d(
                      Units.inchesToMeters(445.35),
                      Units.inchesToMeters(172.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(180)))),
              new AprilTag(
                  4,
                  new Pose3d(
                      Units.inchesToMeters(445.35),
                      Units.inchesToMeters(158.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(180)))),
              new AprilTag(
                  5,
                  new Pose3d(
                      Units.inchesToMeters(469.11),
                      Units.inchesToMeters(135.09),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(270)))),

              // 6-7: Red Trench (Near side to Scoring Table)
              new AprilTag(
                  6,
                  new Pose3d(
                      Units.inchesToMeters(467.64),
                      Units.inchesToMeters(25.37),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(180)))),
              new AprilTag(
                  7,
                  new Pose3d(
                      Units.inchesToMeters(470.59),
                      Units.inchesToMeters(25.37),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(0)))),

              // 8-11: Red Hub (Ring 2)
              new AprilTag(
                  8,
                  new Pose3d(
                      Units.inchesToMeters(483.11),
                      Units.inchesToMeters(135.09),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(270)))),
              new AprilTag(
                  9,
                  new Pose3d(
                      Units.inchesToMeters(492.88),
                      Units.inchesToMeters(144.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(0)))),
              new AprilTag(
                  10,
                  new Pose3d(
                      Units.inchesToMeters(492.88),
                      Units.inchesToMeters(158.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(0)))),
              new AprilTag(
                  11,
                  new Pose3d(
                      Units.inchesToMeters(483.11),
                      Units.inchesToMeters(182.60),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(90)))),

              // 12: Trench, Red (Far side return)
              new AprilTag(
                  12,
                  new Pose3d(
                      Units.inchesToMeters(470.59),
                      Units.inchesToMeters(292.31),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(0)))),

              // 13-14: Red Outpost
              new AprilTag(
                  13,
                  new Pose3d(
                      Units.inchesToMeters(650.92),
                      Units.inchesToMeters(291.47),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(180)))),
              new AprilTag(
                  14,
                  new Pose3d(
                      Units.inchesToMeters(650.92),
                      Units.inchesToMeters(274.47),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(180)))),

              // 15-16: Red Tower
              new AprilTag(
                  15,
                  new Pose3d(
                      Units.inchesToMeters(650.90),
                      Units.inchesToMeters(170.22),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(180)))),
              new AprilTag(
                  16,
                  new Pose3d(
                      Units.inchesToMeters(650.90),
                      Units.inchesToMeters(153.22),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(180)))),

              // --- BLUE ALLIANCE SIDE (IDs 17-32) ---

              // 17: Trench, Blue (Near Scoring Table)
              new AprilTag(
                  17,
                  new Pose3d(
                      Units.inchesToMeters(183.59),
                      Units.inchesToMeters(25.37),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(0)))),

              // 18-21: Blue Hub (Ring 1)
              new AprilTag(
                  18,
                  new Pose3d(
                      Units.inchesToMeters(182.11),
                      Units.inchesToMeters(135.09),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(270)))),
              new AprilTag(
                  19,
                  new Pose3d(
                      Units.inchesToMeters(205.87),
                      Units.inchesToMeters(144.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(0)))),
              new AprilTag(
                  20,
                  new Pose3d(
                      Units.inchesToMeters(205.87),
                      Units.inchesToMeters(158.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(0)))),
              new AprilTag(
                  21,
                  new Pose3d(
                      Units.inchesToMeters(182.11),
                      Units.inchesToMeters(182.60),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(90)))),

              // 22-23: Trench, Blue (Far Side)
              new AprilTag(
                  22,
                  new Pose3d(
                      Units.inchesToMeters(183.59),
                      Units.inchesToMeters(292.31),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(0)))),
              new AprilTag(
                  23,
                  new Pose3d(
                      Units.inchesToMeters(180.64),
                      Units.inchesToMeters(292.31),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(180)))),

              // 24-27: Blue Hub (Ring 2)
              new AprilTag(
                  24,
                  new Pose3d(
                      Units.inchesToMeters(168.11),
                      Units.inchesToMeters(182.60),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(90)))),
              new AprilTag(
                  25,
                  new Pose3d(
                      Units.inchesToMeters(158.34),
                      Units.inchesToMeters(172.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(180)))),
              new AprilTag(
                  26,
                  new Pose3d(
                      Units.inchesToMeters(158.34),
                      Units.inchesToMeters(158.84),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(180)))),
              new AprilTag(
                  27,
                  new Pose3d(
                      Units.inchesToMeters(168.11),
                      Units.inchesToMeters(135.09),
                      Units.inchesToMeters(44.25),
                      new Rotation3d(0, 0, Math.toRadians(270)))),

              // 28: Trench, Blue (Near Side Return)
              new AprilTag(
                  28,
                  new Pose3d(
                      Units.inchesToMeters(180.64),
                      Units.inchesToMeters(25.37),
                      Units.inchesToMeters(35.00),
                      new Rotation3d(0, 0, Math.toRadians(180)))),

              // 29-30: Blue Outpost
              new AprilTag(
                  29,
                  new Pose3d(
                      Units.inchesToMeters(0.30),
                      Units.inchesToMeters(26.22),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(0)))),
              new AprilTag(
                  30,
                  new Pose3d(
                      Units.inchesToMeters(0.30),
                      Units.inchesToMeters(43.22),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(0)))),

              // 31-32: Blue Tower
              new AprilTag(
                  31,
                  new Pose3d(
                      Units.inchesToMeters(0.32),
                      Units.inchesToMeters(147.47),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(0)))),
              new AprilTag(
                  100,
                  new Pose3d(
                      Units.inchesToMeters(0.32),
                      Units.inchesToMeters(164.47),
                      Units.inchesToMeters(21.75),
                      new Rotation3d(0, 0, Math.toRadians(0))))),
          fieldLength,
          fieldWidth);

  /*
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official");

    AprilTagLayoutType(String name) {
      if (layout == null) {
        layoutString = "";
      } else {
        try {
          layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
          throw new RuntimeException(
              "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
        }
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
    */
}

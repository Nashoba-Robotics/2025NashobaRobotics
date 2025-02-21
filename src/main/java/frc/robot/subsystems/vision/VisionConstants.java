// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FrontLeft";
  public static String camera1Name = "FrontRight";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(-0.1524, 0.3429, 0.34925, new Rotation3d(0.0, 0.0, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(0.1524, 0.3429, 0.34925, new Rotation3d(0.0, 0.0, 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  public static final Translation2d centerBlue =
      new Translation2d(Units.inchesToMeters(176.746), aprilTagLayout.getFieldWidth() / 2.0);
  public static final Pose2d rightBlueBranch = new Pose2d(3.20, 3.86, Rotation2d.fromDegrees(0));
  public static final Pose2d leftBlueBranch = new Pose2d(3.20, 4.19, Rotation2d.fromDegrees(0));

  public static final Translation2d centerRed =
      new Translation2d(Units.inchesToMeters(514.129), aprilTagLayout.getFieldWidth() / 2);
  public static final Pose2d rightRedBranch = new Pose2d(11.77, 3.86, Rotation2d.fromDegrees(0));
  public static final Pose2d leftRedBranch = new Pose2d(11.77, 4.19, Rotation2d.fromDegrees(0));

  public static Pose2d[] coralScoringLocations =
      new Pose2d[] {
        rightBlueBranch,
        leftBlueBranch,
        rightBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(60)),
        leftBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(60)),
        rightBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(120)),
        leftBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(120)),
        rightBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(180)),
        leftBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(180)),
        rightBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(240)),
        leftBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(240)),
        rightBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(300)),
        leftBlueBranch.rotateAround(centerBlue, Rotation2d.fromDegrees(300)),
        rightRedBranch,
        leftRedBranch,
        rightRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(60)),
        leftRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(60)),
        rightRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(120)),
        leftRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(120)),
        rightRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(180)),
        leftRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(180)),
        rightRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(240)),
        leftRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(240)),
        rightRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(300)),
        leftRedBranch.rotateAround(centerRed, Rotation2d.fromDegrees(300)),
      };
}

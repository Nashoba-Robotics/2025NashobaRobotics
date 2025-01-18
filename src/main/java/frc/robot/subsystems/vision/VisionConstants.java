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
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Reef tags #6-11 and #17-22
  public static Pose2d[] reefTags =
      new Pose2d[] {
        new Pose2d(
            Units.inchesToMeters(530.49),
            Units.inchesToMeters(130.17),
            Rotation2d.fromDegrees(300)), // 6
        new Pose2d(
            Units.inchesToMeters(546.87),
            Units.inchesToMeters(158.50),
            Rotation2d.fromDegrees(0)), // 7
        new Pose2d(
            Units.inchesToMeters(530.49),
            Units.inchesToMeters(186.83),
            Rotation2d.fromDegrees(60)), // 8
        new Pose2d(
            Units.inchesToMeters(497.77),
            Units.inchesToMeters(186.83),
            Rotation2d.fromDegrees(120)), // 9
        new Pose2d(
            Units.inchesToMeters(481.39),
            Units.inchesToMeters(158.50),
            Rotation2d.fromDegrees(180)), // 10
        new Pose2d(
            Units.inchesToMeters(497.77),
            Units.inchesToMeters(130.17),
            Rotation2d.fromDegrees(240)), // 11
        new Pose2d(
            Units.inchesToMeters(160.39),
            Units.inchesToMeters(130.17),
            Rotation2d.fromDegrees(240)), // 17
        new Pose2d(
            Units.inchesToMeters(144.00),
            Units.inchesToMeters(158.50),
            Rotation2d.fromDegrees(180)), // 18
        new Pose2d(
            Units.inchesToMeters(160.39),
            Units.inchesToMeters(186.83),
            Rotation2d.fromDegrees(120)), // 19
        new Pose2d(
            Units.inchesToMeters(193.10),
            Units.inchesToMeters(186.83),
            Rotation2d.fromDegrees(60)), // 20
        new Pose2d(
            Units.inchesToMeters(209.49),
            Units.inchesToMeters(158.50),
            Rotation2d.fromDegrees(0)), // 21
        new Pose2d(
            Units.inchesToMeters(193.10),
            Units.inchesToMeters(130.17),
            Rotation2d.fromDegrees(300)) // 22
      };

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

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

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}

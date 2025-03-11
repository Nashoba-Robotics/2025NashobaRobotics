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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
      new Transform3d(
          0.1651,
          0.175, // 0.1778
          0.206375,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-30).getRadians(),
              Rotation2d.fromDegrees(-20).getRadians()));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          0.1651,
          -0.175, // -0.1778
          0.206375,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-30).getRadians(),
              Rotation2d.fromDegrees(20).getRadians()));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.05;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.16; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  public static final Pose2d[] centerFaces =
      new Pose2d[] {
        aprilTagLayout.getTagPose(6).get().toPose2d(),
        aprilTagLayout.getTagPose(7).get().toPose2d(),
        aprilTagLayout.getTagPose(8).get().toPose2d(),
        aprilTagLayout.getTagPose(9).get().toPose2d(),
        aprilTagLayout.getTagPose(10).get().toPose2d(),
        aprilTagLayout.getTagPose(11).get().toPose2d(),
        aprilTagLayout.getTagPose(17).get().toPose2d(),
        aprilTagLayout.getTagPose(18).get().toPose2d(),
        aprilTagLayout.getTagPose(19).get().toPose2d(),
        aprilTagLayout.getTagPose(20).get().toPose2d(),
        aprilTagLayout.getTagPose(21).get().toPose2d(),
        aprilTagLayout.getTagPose(22).get().toPose2d(),
      };

  public static double adjustY =
      Units.inchesToMeters(6.469); // offset Y setpoint by distance from tag to reefpost
  public static double adjustX =
      Units.inchesToMeters(36.5 / 2); // offset X setpoint by center of robot to bumber

  public static final Pose2d[] scoringPositions =
      new Pose2d[] {
        centerFaces[0].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[0].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[1].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[1].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[2].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[2].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[3].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[3].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[4].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[4].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[5].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[5].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[6].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[6].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[7].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[7].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[8].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[8].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[9].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[9].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[10].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[10].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
        centerFaces[11].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
        centerFaces[11].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d())),
      };
}

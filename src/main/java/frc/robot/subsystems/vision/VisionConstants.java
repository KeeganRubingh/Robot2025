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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Camera names, must match names configured on coprocessor
  public static String limelightLeftName = "limelight-backl";
  public static String limelightRightName = "limelight-backr";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCameraLeft =
  new Transform3d(
    //IP 10.9.30.16
    //back Left (on endgame) values on robot, 
    // LL Forward -0.2599(10.25in), LL Right -0.2757(-10.854in), LL up 0.3161(12.96in - 0.5in), LL Roll 0, LL Pitch -10, LL Yaw -172.5
    //LL RIGHT IS +, IN CODE(y value) IT'S -
      -0.2599,
      0.2757, 
      .3161, /*  .3291 - 0.13(0.5 in) */  
      //LL PITCH IS +, IN CODE IT'S -
      new Rotation3d(0.0, Math.toRadians(10.0), Math.toRadians(172.5)));
public static Transform3d robotToCameraRight =
  new Transform3d(
    // 10.9.30.15
    //back right values on robot, 
    // LL Forward -0.301(-11.844in), LL Right 0.269(10.6in), LL up 0.228(8.982in), LL Roll 0, LL Pitch 25, LL Yaw 165.
    //LL RIGHT IS -, IN CODE(y value) IT'S +
      -0.301,
      -0.269,
      0.228,
      //LL PITCH IS +, IN CODE IT'S -
      new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(165.0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines(0.01), for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 1.0; // Meters, 0.01 was original value was hyper sensitive.
  public static double angularStdDevBaseline = Double.POSITIVE_INFINITY;

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

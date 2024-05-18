// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public record InterpolationData(
    Pose2d measuredPose, Pose2d visionPose, Transform2d transform, String label) {
  public InterpolationData(Pose2d measuredPose, Pose2d visionPose, String label) {
    this(measuredPose, visionPose, new Transform2d(visionPose, measuredPose), label);
  }
}

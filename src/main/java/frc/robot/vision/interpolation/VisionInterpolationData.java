// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Translation2d;

public record VisionInterpolationData(
    Translation2d measuredPose, Translation2d visionPose, String label) {}

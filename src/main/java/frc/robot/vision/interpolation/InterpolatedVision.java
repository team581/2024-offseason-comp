// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;

public class InterpolatedVision {
  private static final InterpolatedVisionDataset usedSet =
      RobotConfig.get().vision().interpolatedVisionSet();

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d interpolatePose(Pose2d visionInput) {
    var usedDataPoints = FmsSubsystem.isRedAlliance() ? usedSet.redSet : usedSet.blueSet;

    return new Pose2d(
        InterpolationUtil.interpolateTranslation(usedDataPoints, visionInput.getTranslation()),
        visionInput.getRotation());
  }

  private InterpolatedVision() {}
}

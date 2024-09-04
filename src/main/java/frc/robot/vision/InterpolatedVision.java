// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.fms.FmsSubsystem;
import java.util.List;

public class InterpolatedVision {
  private static final VisionInterpolationData SUBWOOFER =
      new VisionInterpolationData(
          new Translation2d(15.2245, 5.522), new Translation2d(15.194, 5.634), "SUBWOOFER");
  private static final VisionInterpolationData PODIUM_SPEAKER_INTERSECTION =
      new VisionInterpolationData(
          new Translation2d(13.0745, 5.522),
          new Translation2d(13.125, 5.722),
          "PODIUM_SPEAKER_INTERSECTION");
  private static final VisionInterpolationData WING_LINE_MIDDLE =
      new VisionInterpolationData(
          new Translation2d(11.059, 6.842), new Translation2d(11.16, 6.845), "WING_LINE_MIDDLE");

  private static final VisionInterpolationData FRONT_PODIUM_MIDDLE =
      new VisionInterpolationData(
          new Translation2d(13.799, 4.202),
          new Translation2d(13.905, 4.361),
          "FRONT_PODIUM_MIDDLE");

  private static final List<VisionInterpolationData> DATA_POINTS_RED =
      List.of(SUBWOOFER, PODIUM_SPEAKER_INTERSECTION, WING_LINE_MIDDLE, FRONT_PODIUM_MIDDLE);

  private static final List<VisionInterpolationData> DATA_POINTS_BLUE = List.of();

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d interpolatePose(Pose2d visionInput) {
    var usedDataPoints = FmsSubsystem.isRedAlliance() ? DATA_POINTS_RED : DATA_POINTS_BLUE;

    return new Pose2d(
        InterpolationUtil.interpolateTranslation(usedDataPoints, visionInput.getTranslation()),
        visionInput.getRotation());
  }

  private InterpolatedVision() {}
}

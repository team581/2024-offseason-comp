// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class VisionUtil {
  private static final VisionInterpolationData SUBWOOFER =
      new VisionInterpolationData(
          new Pose2d(15.279, 5.522, new Rotation2d(0)),
          new Pose2d(15.289, 5.587, new Rotation2d(0)),
          "Subwoofer");
  private static final VisionInterpolationData WING_LINE_AMP =
      new VisionInterpolationData(
          new Pose2d(11.232, 7.78, new Rotation2d(0)),
          new Pose2d(11.4, 7.865, new Rotation2d(0)),
          "AMP_SIDE_STAGE");

  private static final VisionInterpolationData STAGE_FRONT =
      new VisionInterpolationData(
          new Pose2d(12.85, 4.71, new Rotation2d(0)),
          new Pose2d(12.83, 4.66, new Rotation2d(0)),
          "StageFront");
  private static final VisionInterpolationData STAGE_MIDDLE =
      new VisionInterpolationData(
          new Pose2d(11.98, 5.16, new Rotation2d(0)),
          new Pose2d(12.0, 5.11, new Rotation2d(0)),
          "StageMiddle");

  private static final List<VisionInterpolationData> DATA_POINTS =
      List.of(SUBWOOFER, WING_LINE_AMP);

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d interpolatePose(Pose2d visionInput) {
    double distanceSum = 0;
    for (var dataPoint : DATA_POINTS) {
      double distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());

      distanceSum += distancePoint;
    }
    double weightedX = 0;
    double weightedY = 0;

    for (var dataPoint : DATA_POINTS) {
      double distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());
      var weight = 1 - ((distanceSum - distancePoint) / distanceSum);
      var result = dataPoint.visionPose().minus(dataPoint.measuredPose()).times(weight);
      weightedX += result.getX();
      weightedY += result.getY();
    }
    Pose2d interpolatedSum =
        new Pose2d(
            visionInput.getX() + weightedX,
            visionInput.getY() + weightedY,
            visionInput.getRotation());
    return interpolatedSum;
  }
}

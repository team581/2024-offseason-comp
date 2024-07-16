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
          new Pose2d(15.19, 5.54, new Rotation2d(0)),
          new Pose2d(15.23, 5.45, new Rotation2d(0)),
          "Subwoofer");
  private static final VisionInterpolationData STAGE_FRONT =
      new VisionInterpolationData(
          new Pose2d(12.85, 4.71, new Rotation2d(0)),
          new Pose2d(12.83, 4.66, new Rotation2d(0)),
          "StageFront");
  private static final VisionInterpolationData AMP_SIDE_STAGE =
      new VisionInterpolationData(
          new Pose2d(10.27, 7.79, new Rotation2d(0)),
          new Pose2d(10.36, 7.74, new Rotation2d(0)),
          "AMP_SIDE_STAGE");
  private static final VisionInterpolationData STAGE_MIDDLE =
      new VisionInterpolationData(
          new Pose2d(17.0, 9.7, new Rotation2d(0)),
          new Pose2d(12.0, 5.0, new Rotation2d(0)),
          "StageMiddle");

  private static final List<VisionInterpolationData> DATA_POINTS =
      List.of(SUBWOOFER, STAGE_FRONT, AMP_SIDE_STAGE, STAGE_MIDDLE);

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d interpolatePose(Pose2d visionInput) {
    double distanceSum = 0;
    double distancePoint = 0;
    for (var dataPoint : DATA_POINTS) {
      distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());

      distanceSum += distancePoint;
    }
    double weightedX = 0;
    double weightedY = 0;
    Rotation2d weightedRotation = new Rotation2d();
    for (var dataPoint : DATA_POINTS) {
      distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());
      var weight = 1 - ((distanceSum - distancePoint) / distanceSum);
      var result = dataPoint.visionPose().minus(dataPoint.measuredPose()).times(weight);
      weightedX += result.getX();
      weightedY += result.getY();
      weightedRotation = weightedRotation.plus(result.getRotation());
    }
    Pose2d interpolatedSum =
        new Pose2d(
            visionInput.getX() + weightedX,
            visionInput.getY() + weightedY,
            visionInput.getRotation().plus(weightedRotation));
    return interpolatedSum;
  }
}

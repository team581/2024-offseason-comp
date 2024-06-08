// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class InterpolationUtil {
  private static final InterpolationData SUBWOOFER = new InterpolationData(null, null, "Subwoofer");
  private static final InterpolationData STAGE_FRONT =
      new InterpolationData(null, null, "StageFront");
  private static final InterpolationData STAGE_RIGHT =
      new InterpolationData(null, null, "StageRight");
  private static final InterpolationData STAGE_MIDDLE =
      new InterpolationData(null, null, "StageMiddle");

  private static final List<InterpolationData> DATA_POINTS =
      List.of(SUBWOOFER, STAGE_FRONT, STAGE_RIGHT, STAGE_MIDDLE);

  public static Pose2d interpolatePose(Pose2d visionInput) {
    double distanceSum = 0;

    for (var dataPoint : DATA_POINTS) {
      var distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());

      distanceSum += distancePoint;
    }
    double weightedX = 0;
    double weightedY = 0;
    Rotation2d weightedRotation = new Rotation2d();
    for (var dataPoint : DATA_POINTS) {
      var distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());

      var result =
          dataPoint.visionPose().minus(dataPoint.measuredPose()).times(distanceSum - distancePoint);
      weightedX += result.getX();
      weightedY += result.getY();
      weightedRotation = weightedRotation.plus(result.getRotation());
    }
    Pose2d weightedSum = new Pose2d(weightedX, weightedY, weightedRotation);

    return weightedSum.div(distanceSum);
  }
}

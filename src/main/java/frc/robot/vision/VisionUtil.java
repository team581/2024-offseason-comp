// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

// 260.55cm

// 13.9735

// 2.62 meters

public class VisionUtil {
  private static final VisionInterpolationData SUBWOOFER =
      new VisionInterpolationData(
          // output before changing math, with just subwoofer point:   15.162,     5.749
          // output after changing math, with just subwoofer point: 15.222, 5.524
          new Pose2d(15.2245, 5.522, new Rotation2d(0)),
          new Pose2d(15.194, 5.634, new Rotation2d(0)),
          "SUBWOOFER");
  private static final VisionInterpolationData PODIUM_SPEAKER_INTERSECTION =
      // output with both subwoofer and intersection: 12.962, 5.623
      // output with only intersection, after redoing math:
      new VisionInterpolationData(
          new Pose2d(13.0745, 5.522, new Rotation2d(0)),
          new Pose2d(13.125, 5.722, new Rotation2d(0)),
          "PODIUM_SPEAKER_INTERSECTION");
  private static final VisionInterpolationData WING_LINE_MIDDLE =
      // output with both subwoofer and intersection: 12.962, 5.623
      // output with only intersection, after redoing math:
      new VisionInterpolationData(
          new Pose2d(11.059, 6.842, new Rotation2d(0)),
          new Pose2d(11.16, 6.845, new Rotation2d(0)),
          "WING_LINE_MIDDLE");
  private static final double SENSITIVITY = 2;
  private static final List<VisionInterpolationData> DATA_POINTS =
      List.of(SUBWOOFER, PODIUM_SPEAKER_INTERSECTION, WING_LINE_MIDDLE);

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d interpolatePose(Pose2d visionInput) {
    DogLog.log("Debug/IntersectionMeasuredPose", PODIUM_SPEAKER_INTERSECTION.measuredPose());

    double unnormalizedWeightsSum = 0;
    for (var dataPoint : DATA_POINTS) {
      double distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());

      var weight = 1 / Math.pow(distancePoint, SENSITIVITY);

      unnormalizedWeightsSum += weight;
    }
    double weightedX = 0;
    double weightedY = 0;



    for (var dataPoint : DATA_POINTS) {
      double distancePoint =
          dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());
         var weight = (1 / Math.pow(distancePoint, SENSITIVITY)) / unnormalizedWeightsSum;
      DogLog.log("Debug/" + dataPoint.label() + "/Weight", weight);
      var result = dataPoint.measuredPose().minus(dataPoint.visionPose()).times(weight);
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

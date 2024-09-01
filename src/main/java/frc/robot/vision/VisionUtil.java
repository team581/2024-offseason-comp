// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class VisionUtil {
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

  private static final List<VisionInterpolationData> DATA_POINTS =
      List.of(SUBWOOFER, PODIUM_SPEAKER_INTERSECTION, WING_LINE_MIDDLE);

  public static Pose2d interpolatePose(Pose2d visionInput) {
    return new Pose2d(
        interpolateTranslation(visionInput.getTranslation()), visionInput.getRotation());
  }

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Translation2d interpolateTranslation(Translation2d visionInput) {
    double unnormalizedWeightsSum =
        DATA_POINTS.stream()
            .mapToDouble(dataPoint -> dataPoint.visionPose().getDistance(visionInput))
            .sum();

    double weightedX = 0;
    double weightedY = 0;

    for (var dataPoint : DATA_POINTS) {
      double distancePoint = dataPoint.visionPose().getDistance(visionInput);
      var weight = calculateUnnormalizedWeight(distancePoint) / unnormalizedWeightsSum;
      var result = dataPoint.measuredPose().minus(dataPoint.visionPose()).times(weight);
      weightedX += result.getX();
      weightedY += result.getY();
    }

    return new Translation2d(visionInput.getX() + weightedX, visionInput.getY() + weightedY);
  }

  private static double calculateUnnormalizedWeight(double distance) {
    return 1.0 / Math.pow(distance, 2);
  }

  private VisionUtil() {}
}

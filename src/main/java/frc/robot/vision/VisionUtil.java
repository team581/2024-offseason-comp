// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.List;

public class VisionUtil {
  private static final VisionInterpolationData SUBWOOFER =
      new VisionInterpolationData(
          new Pose2d(20.2, 10.5, new Rotation2d(0)),
          new Pose2d(15.2, 5.5, new Rotation2d(0)),
          "Subwoofer");
  // private static final VisionInterpolationData STAGE_FRONT =
  //     new VisionInterpolationData(
  //         new Pose2d(18.0, 10.5,new Rotation2d(0)), new Pose2d(13.0, 5.5, new Rotation2d(0)),
  // "StageFront");
  private static final VisionInterpolationData AMP_SIDE_STAGE =
      new VisionInterpolationData(
          new Pose2d(5.7, 3.0, new Rotation2d(0)),
          new Pose2d(10.7, 8.0, new Rotation2d(0)),
          "AMP_SIDE_STAGE");
  // private static final VisionInterpolationData STAGE_MIDDLE =
  //     new VisionInterpolationData(
  //         new Pose2d(17.0, 9.7, new Rotation2d(0)), new Pose2d(12.0, 4.7, new Rotation2d(0)),
  // "StageMiddle");

  private static final List<VisionInterpolationData> DATA_POINTS =
      List.of(SUBWOOFER, AMP_SIDE_STAGE);

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d interpolatePose(Pose2d visionInput) {
    double distanceSum = 0;
    double distancePoint = 0;
    Transform2d result = new Transform2d();
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
      result = dataPoint.visionPose().minus(dataPoint.measuredPose()).times(weight);
      weightedX += result.getX();
      weightedY += result.getY();
      weightedRotation = weightedRotation.plus(result.getRotation());
    }
    Pose2d weightedSum = new Pose2d(weightedX, weightedY, weightedRotation);
    DogLog.log("VisionUtil/distanceSum", distanceSum);
    DogLog.log("VisionUtil/distancePoint", distancePoint);
    DogLog.log("VisionUtil/weightedSum", weightedSum);
    DogLog.log("Result", result);

    return weightedSum;
  }
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.vision;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import java.util.List;

// public class VisionUtil {
//   private static final VisionInterpolationData SUBWOOFER =
//       new VisionInterpolationData(new Pose2d(null, 89,5.5, new Rotation2d(67)), "Subwoofer");
//   private static final VisionInterpolationData STAGE_FRONT =
//       new VisionInterpolationData(null, null, "StageFront");
//   private static final VisionInterpolationData STAGE_RIGHT =
//       new VisionInterpolationData(null, null, "StageRight");
//   private static final VisionInterpolationData STAGE_MIDDLE =
//       new VisionInterpolationData(null, null, "StageMiddle");

//   private static final List<VisionInterpolationData> DATA_POINTS =
//       List.of(SUBWOOFER, STAGE_FRONT, STAGE_RIGHT, STAGE_MIDDLE);

//   /**
//    * @param visionInput - pose from the limelight
//    * @return a transformed pose that can be added to the pose estimator
//    */
//   public static Pose2d interpolatePose(Pose2d visionInput) {
//     double distanceSum = 0;

//     for (var dataPoint : DATA_POINTS) {
//       var distancePoint =
//           dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());

//       distanceSum += distancePoint;
//     }
//     double weightedX = 0;
//     double weightedY = 0;
//     Rotation2d weightedRotation = new Rotation2d();
//     for (var dataPoint : DATA_POINTS) {
//       var distancePoint =
//           dataPoint.visionPose().getTranslation().getDistance(visionInput.getTranslation());

//       var result =
//           dataPoint.visionPose().minus(dataPoint.measuredPose()).times(distanceSum - distancePoint);
//       weightedX += result.getX();
//       weightedY += result.getY();
//       weightedRotation = weightedRotation.plus(result.getRotation());
//     }
//     Pose2d weightedSum = new Pose2d(weightedX, weightedY, weightedRotation);

//     return weightedSum.div(distanceSum);
//   }
// }

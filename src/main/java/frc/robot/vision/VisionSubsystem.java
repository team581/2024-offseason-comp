// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;

public class VisionSubsystem extends LifecycleSubsystem {
  private static final double FLOOR_SPOT_MAX_DISTANCE_FOR_SUBWOOFER = 14.0;
  private static final boolean SHOOT_TO_SIDE_ENABLED = true;
  public static final boolean LIMELIGHT_UPSIDE_DOWN = true;

  public static final Pose2d ORIGINAL_RED_SPEAKER =
      new Pose2d(
          Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
  public static final Pose2d ORIGINAL_BLUE_SPEAKER =
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));

  public static final Pose2d RED_FLOOR_SPOT_SUBWOOFER =
      new Pose2d(15.5, 7.9, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_FLOOR_SPOT_SUBWOOFER =
      new Pose2d(1, 7.9, Rotation2d.fromDegrees(0));

  public static final Pose2d RED_FLOOR_SPOT_AMP_AREA =
      new Pose2d(9.5, 7, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_FLOOR_SPOT_AMP_AREA = new Pose2d(7, 7, Rotation2d.fromDegrees(0));

  private static final Pose2d RED_FLOOR_SHOT_ROBOT_FALLBACK_POSE =
      new Pose2d(new Translation2d(16.54 / 2.0 - 0.5, 1.25), new Rotation2d());
  private static final Pose2d BLUE_FLOOR_SHOT_ROBOT_FALLBACK_POSE =
      new Pose2d(new Translation2d(16.54 / 2.0 + 0.5, 1.25), new Rotation2d());

  private final Timer limelightTimer = new Timer();
  private double limelightHeartbeat = -1;

  private final InterpolatingDoubleTreeMap distanceToDev = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap angleToSideShotOffset = new InterpolatingDoubleTreeMap();

  private final ImuSubsystem imu;

  private Optional<VisionResult> getRawVisionResult() {
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    if (estimatePose == null) {
      return Optional.empty();
    }

    if (estimatePose.tagCount == 0) {
      return Optional.empty();
    }

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (estimatePose.pose.getX() == 0.0 && estimatePose.pose.getY() == 0.0) {
      return Optional.empty();
    }

    return Optional.of(new VisionResult(estimatePose.pose, estimatePose.timestampSeconds));
  }

  /**
   * @return an interpolated vision pose, ready to be added to the estimator
   */
  public Optional<VisionResult> getVisionResult() {
    var maybeRawData = getRawVisionResult();

    if (RobotConfig.get().perfToggles().interpolatedVision() && maybeRawData.isPresent()) {
      var rawData = maybeRawData.get();

      return Optional.of(
          new VisionResult(
              InterpolatedVision.interpolatePose(rawData.pose()), rawData.timestamp()));
    }

    // No raw data to operate on
    return maybeRawData;
  }

  public static DistanceAngle distanceAngleToTarget(Pose2d target, Pose2d current) {
    double distance = target.getTranslation().getDistance(current.getTranslation());
    double angle =
        Units.radiansToDegrees(
            Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));

    return new DistanceAngle(distance, angle, false);
  }

  private Pose2d robotPose = new Pose2d();

  public VisionSubsystem(ImuSubsystem imu) {
    super(SubsystemPriority.VISION);
    this.imu = imu;

    distanceToDev.put(1.0, 0.4);
    distanceToDev.put(5.0, 6.0);
    distanceToDev.put(3.8, 2.6);
    distanceToDev.put(4.5, 3.0);
    distanceToDev.put(3.37, 2.45);
    distanceToDev.put(7.0, 10.0);

    angleToSideShotOffset.put(Units.degreesToRadians(100), Units.degreesToRadians(5.0));
    angleToSideShotOffset.put(Units.degreesToRadians(40), Units.degreesToRadians(1.75));
    angleToSideShotOffset.put(Units.degreesToRadians(30), Units.degreesToRadians(0.0));
    angleToSideShotOffset.put(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0));

    limelightTimer.start();
  }

  private static Pose2d getSpeaker() {
    if (FmsSubsystem.isRedAlliance()) {
      return ORIGINAL_RED_SPEAKER;
    } else {
      return ORIGINAL_BLUE_SPEAKER;
    }
  }

  public DistanceAngle getDistanceAngleSpeaker() {
    Pose2d speakerPose = getSpeaker();
    DistanceAngle distanceAngleToSpeaker = distanceAngleToTarget(speakerPose, robotPose);
    return adjustForSideShot(distanceAngleToSpeaker);
  }

  private DistanceAngle adjustForSideShot(DistanceAngle originalPosition) {
    if (!SHOOT_TO_SIDE_ENABLED) {
      return originalPosition;
    }

    double rawAngleDegrees = originalPosition.targetAngle();

    if (!FmsSubsystem.isRedAlliance()) {
      rawAngleDegrees += 180;
    }
    double angleDegrees = MathUtil.inputModulus(rawAngleDegrees, -180.0, 180.0);

    double absoluteOffsetRadians =
        (angleToSideShotOffset.get(Units.degreesToRadians(Math.abs(angleDegrees))));
    double offsetRadians = Math.copySign(absoluteOffsetRadians, angleDegrees);
    DogLog.log("Vision/ShootToTheSide/Offset", Units.radiansToDegrees(offsetRadians));

    var adjustedAngle =
        Units.radiansToDegrees(
            Units.degreesToRadians(originalPosition.targetAngle()) + offsetRadians);

    return new DistanceAngle(
        originalPosition.distance(), adjustedAngle, originalPosition.seesSpeakerTag());
  }

  public DistanceAngle getDistanceAngleFloorShot() {
    Pose2d goalPoseSubwoofer;
    Pose2d goalPoseAmpArea;
    Pose2d fallbackPose;
    if (FmsSubsystem.isRedAlliance()) {
      goalPoseSubwoofer = RED_FLOOR_SPOT_SUBWOOFER;
      goalPoseAmpArea = RED_FLOOR_SPOT_AMP_AREA;
      fallbackPose = RED_FLOOR_SHOT_ROBOT_FALLBACK_POSE;
    } else {
      goalPoseSubwoofer = BLUE_FLOOR_SPOT_SUBWOOFER;
      goalPoseAmpArea = BLUE_FLOOR_SPOT_AMP_AREA;
      fallbackPose = BLUE_FLOOR_SHOT_ROBOT_FALLBACK_POSE;
    }

    fallbackPose = new Pose2d(fallbackPose.getTranslation(), robotPose.getRotation());
    var usedRobotPose = getState() == VisionState.OFFLINE ? fallbackPose : robotPose;

    var subwooferSpotDistance = distanceAngleToTarget(goalPoseSubwoofer, usedRobotPose);
    var usedGoalPose = goalPoseSubwoofer;
    var result = subwooferSpotDistance;

    if (subwooferSpotDistance.distance() > FLOOR_SPOT_MAX_DISTANCE_FOR_SUBWOOFER) {
      result = distanceAngleToTarget(goalPoseAmpArea, usedRobotPose);
      usedGoalPose = goalPoseAmpArea;
      result = new DistanceAngle(581.0, result.targetAngle(), false);
    }

    if (RobotConfig.IS_DEVELOPMENT) {
      DogLog.log("Vision/FloorShot/SubwooferPose", goalPoseSubwoofer);
      DogLog.log("Vision/FloorShot/AmpAreaPose", goalPoseAmpArea);
    }
    DogLog.log("Vision/FloorShot/UsedTargetPose", usedGoalPose);

    return result;
  }

  public double getStandardDeviation(double distance) {
    return distanceToDev.get(distance);
  }

  public void setRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  @Override
  public void robotPeriodic() {
    if (FmsSubsystem.isRedAlliance()) {
      LimelightHelpers.setPriorityTagID("", 4);
    } else {
      LimelightHelpers.setPriorityTagID("", 7);
    }
    DogLog.log("Vision/State", getState());

    var newHeartbeat = LimelightHelpers.getLimelightNTDouble("", "hb");

    if (limelightHeartbeat == newHeartbeat) {
      // No new data, Limelight dead?
    } else {
      limelightTimer.restart();
    }

    limelightHeartbeat = newHeartbeat;

    LimelightHelpers.SetRobotOrientation(
        "",
        imu.getRobotHeading(),
        imu.getRobotAngularVelocity(),
        imu.getPitch(),
        imu.getPitchRate(),
        imu.getRoll(),
        imu.getRollRate());
  }

  public VisionState getState() {
    if (limelightTimer.hasElapsed(5)) {
      // Heartbeat hasn't updated
      return VisionState.OFFLINE;
    }

    // getVisionResult() returns empty if there's no seen tags
    if (getVisionResult().isPresent()) {
      return VisionState.SEES_TAGS;
    }
    return VisionState.ONLINE_NO_TAGS;
  }
}

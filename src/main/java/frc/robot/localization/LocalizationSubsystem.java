// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.VisionSubsystem;

public class LocalizationSubsystem extends LifecycleSubsystem {
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private final VisionSubsystem vision;
  private double lastAddedVisionTimestamp = 0;

  private final TimeInterpolatableBuffer<Pose2d> poseHistory =
      TimeInterpolatableBuffer.createBuffer(1.5);

  public LocalizationSubsystem(SwerveSubsystem swerve, ImuSubsystem imu, VisionSubsystem vision) {
    super(SubsystemPriority.LOCALIZATION);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;
    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveSubsystem.KINEMATICS,
            imu.getRobotHeading(),
            swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
            new Pose2d());
    odometry =
        new SwerveDriveOdometry(
            SwerveSubsystem.KINEMATICS,
            imu.getRobotHeading(),
            swerve.getModulePositions().toArray(new SwerveModulePosition[4]));
  }

  @Override
  public void robotPeriodic() {
    SwerveModulePosition[] modulePositions =
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]);
    odometry.update(imu.getRobotHeading(), modulePositions);
    poseEstimator.update(imu.getRobotHeading(), modulePositions);

    var maybeResults = vision.getVisionResult();

    if (maybeResults.isPresent()) {
      var results = maybeResults.get();
      Pose2d visionPose = results.pose();

      double visionTimestamp = results.timestamp();

      if (visionTimestamp == lastAddedVisionTimestamp) {
        // Don't add the same vision pose over and over
      } else {
        poseEstimator.addVisionMeasurement(
            visionPose,
            visionTimestamp,
            VecBuilder.fill(
                RobotConfig.get().vision().xyStdDev(),
                RobotConfig.get().vision().xyStdDev(),
                RobotConfig.get().vision().thetaStdDev()));
        lastAddedVisionTimestamp = visionTimestamp;
      }
    }

    poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());

    DogLog.log("Localization/OdometryPose", getOdometryPose());
    DogLog.log("Localization/EstimatedPose", getPose());
    PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    if (mt2Estimate != null) {
      DogLog.log("Localization/LimelightPoseRaw", mt2Estimate.pose);
    }

    vision.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // get pose at timestamp method
  public Pose2d getPose(double timestamp) {
    return poseHistory.getSample(timestamp).orElseGet(this::getPose);
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    resetPose(pose, pose);
  }

  public void resetPose(Pose2d estimatedPose, Pose2d odometryPose) {
    imu.setAngle(odometryPose.getRotation());
    poseEstimator.resetPosition(
        estimatedPose.getRotation(),
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
        estimatedPose);
    odometry.resetPosition(
        odometryPose.getRotation(),
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
        odometryPose);
  }

  private void resetGyro(Rotation2d gyroAngle) {
    Pose2d estimatedPose = new Pose2d(getPose().getTranslation(), gyroAngle);
    Pose2d odometryPose = new Pose2d(getOdometryPose().getTranslation(), gyroAngle);
    resetPose(estimatedPose, odometryPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 0 : 180)));
  }
}

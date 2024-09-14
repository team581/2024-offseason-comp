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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private double lookaheadTime = 0.2;
  private boolean useLookahead = false;

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
            Rotation2d.fromDegrees(imu.getRobotHeading()),
            swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
            new Pose2d());
    odometry =
        new SwerveDriveOdometry(
            SwerveSubsystem.KINEMATICS,
            Rotation2d.fromDegrees(imu.getRobotHeading()),
            swerve.getModulePositions().toArray(new SwerveModulePosition[4]));
  }

  @Override
  public void robotPeriodic() {
    SwerveModulePosition[] modulePositions =
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]);
    odometry.update(Rotation2d.fromDegrees(imu.getRobotHeading()), modulePositions);
    poseEstimator.update(Rotation2d.fromDegrees(imu.getRobotHeading()), modulePositions);

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
    DogLog.log("Localization/EstimatedPose/UsedPose", getUsedPose());
    DogLog.log("Localization/EstimatedPose/CurrentPose", getPose());
    DogLog.log("Localization/EstimatedPose/LookaheadPose", getLookaheadPose());
    PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    if (mt2Estimate != null) {
      DogLog.log("Localization/LimelightPoseRaw", mt2Estimate.pose);
    }

    vision.setRobotPose(getUsedPose());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getUsedPose() {
    return getLookaheadRobot(useLookahead);
  }

  public Pose2d getLookaheadPose() {
    return getLookaheadRobot(true);
  }

  // get pose at timestamp method
  public Pose2d getPose(double timestamp) {
    return poseHistory.getSample(timestamp).orElseGet(this::getUsedPose);
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    resetPose(pose, pose);
  }

  public void resetPose(Pose2d estimatedPose, Pose2d odometryPose) {
    imu.setAngle(odometryPose.getRotation().getDegrees());
    poseEstimator.resetPosition(
        estimatedPose.getRotation(),
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
        estimatedPose);
    odometry.resetPosition(
        odometryPose.getRotation(),
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
        odometryPose);
  }

  private void resetGyro(double gyroAngle) {
    Pose2d estimatedPose =
        new Pose2d(getUsedPose().getTranslation(), Rotation2d.fromDegrees(gyroAngle));
    Pose2d odometryPose =
        new Pose2d(getOdometryPose().getTranslation(), Rotation2d.fromDegrees(gyroAngle));
    resetPose(estimatedPose, odometryPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(() -> resetGyro(FmsSubsystem.isRedAlliance() ? 0 : 180));
  }

  public Pose2d getLookaheadRobot(boolean doLookahead) {
    if (!doLookahead) {
      return getPose();
    }

    ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();
    Pose2d accel = new Pose2d(imu.getXAcceleration(), imu.getYAcceleration(), new Rotation2d(0));
    Pose2d vel =
        new Pose2d(
            accel.getX() * lookaheadTime + robotSpeeds.vxMetersPerSecond,
            accel.getY() * lookaheadTime + robotSpeeds.vyMetersPerSecond,
            Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond));
    Pose2d lookahead =
        new Pose2d(
            vel.getX() * lookaheadTime + getPose().getX(),
            vel.getY() * lookaheadTime + getPose().getY(),
            vel.getRotation().times(lookaheadTime).plus(getPose().getRotation()));

    return lookahead;
  }
}

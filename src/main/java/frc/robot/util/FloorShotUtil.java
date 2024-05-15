// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionSubsystem;

public class FloorShotUtil {
  public final SwerveSubsystem swerve;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;

  private final double TimeConstant = 0.02;

  public FloorShotUtil(SwerveSubsystem swerve, LocalizationSubsystem localization, VisionSubsystem vision) {
    this.swerve = swerve;
    this.localization = localization;
    this.vision = vision;
  }

  public Transform2d getRobotVector() {
    var chassisSpeeds = swerve.getFieldRelativeSpeeds();
    return new Transform2d(new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond), new Rotation2d(chassisSpeeds.omegaRadiansPerSecond));
  }

  public Pose2d getLookAheadPose() {
    var chassisSpeeds = getRobotVector();
    Pose2d currentPose = localization.getPose();
    double lookAheadX = currentPose.getX() + (chassisSpeeds.getX() * TimeConstant);
    double lookAheadY = currentPose.getY() + (chassisSpeeds.getY() * TimeConstant);
    double lookAheadOmega = currentPose.getRotation().getRadians() + (chassisSpeeds.getRotation().getRadians() * TimeConstant);

    return new Pose2d(lookAheadX, lookAheadY, new Rotation2d(lookAheadOmega));
  }

  public Translation2d getLookAheadNoteVector() {
    double lookAheadDistanceToFloorSpot = vision.getDistanceAngleFloorShot().distance();

    return new Translation2d();
  }
}

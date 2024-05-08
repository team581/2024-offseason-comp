// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.swerve.SwerveSubsystem;

public class FloorShotUtil {
  public final SwerveSubsystem swerve;

  public FloorShotUtil(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  public Translation2d chassisSpeedsToVector() {
    var chassisSpeeds = swerve.getFieldRelativeSpeeds();
    return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }
}

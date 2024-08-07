// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;
import java.util.function.Supplier;

public record AutoNoteStep(Supplier<Pose2d> noteSearchPose, AutoNoteAction action) {
  public static final AutoNoteStep CLEANUP = new AutoNoteStep(() -> null, AutoNoteAction.CLEANUP);

  public static Pose2d noteIdToPose(int noteId) {
    switch (noteId) {
      case 1:
        if (FmsSubsystem.isRedAlliance()) {
          return new Pose2d(13.65, 4.106, new Rotation2d(0));
        } else {
          return new Pose2d(2.896, 4.106, new Rotation2d(0));
        }
      case 2:
        if (FmsSubsystem.isRedAlliance()) {
          return new Pose2d(13.65, 5.553, new Rotation2d(0));
        } else {
          return new Pose2d(2.896, 5.553, new Rotation2d(0));
        }
      case 3:
        if (FmsSubsystem.isRedAlliance()) {
          return new Pose2d(13.65, 7.001, new Rotation2d(0));
        } else {
          return new Pose2d(2.896, 7.001, new Rotation2d(0));
        }
        // Subtract 0.5 meters from the x to put back on midline
      case 4:
        return new Pose2d(8.771, 7.458, new Rotation2d(0));
      case 5:
        return new Pose2d(8.771, 5.782, new Rotation2d(0));
      case 6:
        return new Pose2d(8.771, 4.106, new Rotation2d(0));
      case 7:
        return new Pose2d(8.771, 2.429, new Rotation2d(0));
      case 8:
        return new Pose2d(8.771, 0.753, new Rotation2d(0));
      default:
        throw new IllegalArgumentException("Expected a note ID from between 1 and 8");
    }
  }

  public AutoNoteStep(int noteId, AutoNoteAction action) {
    this(() -> noteIdToPose(noteId), action);
  }
}

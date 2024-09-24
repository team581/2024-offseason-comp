// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.fms.FmsSubsystem;
import java.util.Optional;

public class AutoNoteStaged {
  public static Translation2d noteIdToTranslation(int noteId) {
    switch (noteId) {
      case 1:
        if (FmsSubsystem.isRedAlliance()) {
          return new Translation2d(13.65, 4.106);
        } else {
          return new Translation2d(2.896, 4.106);
        }
      case 2:
        if (FmsSubsystem.isRedAlliance()) {
          return new Translation2d(13.65, 5.553);
        } else {
          return new Translation2d(2.896, 5.553);
        }
      case 3:
        if (FmsSubsystem.isRedAlliance()) {
          return new Translation2d(13.65, 7.001);
        } else {
          return new Translation2d(2.896, 7.001);
        }

        // TODO: SUBTRACT 1 METER FROM X TO PUT INTO NORMAL FIELD POSES
      case 4:
        return new Translation2d(8.271, 7.458);
      case 5:
        return new Translation2d(8.271, 5.782);
      case 6:
        return new Translation2d(8.271, 4.106);
      case 7:
        return new Translation2d(8.271, 2.429);
      case 8:
        return new Translation2d(8.271, 0.753);
      default:
        throw new IllegalArgumentException("Expected a note ID from between 1 and 8");
    }
  }

  private final int id;

  public AutoNoteStaged(int id) {
    this.id = id;
  }

  public Optional<Translation2d> getPose() {
    return Optional.of(noteIdToTranslation(id));
  }
}

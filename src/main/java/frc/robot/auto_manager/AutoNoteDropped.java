// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class AutoNoteDropped {
  private static int nextDroppedNoteId = 0;

  public static void clearDroppedNotes() {
    droppedNoteIdToPose.clear();
    nextDroppedNoteId = 0;
  }

  public static void addDroppedNote(Pose2d pose) {
    droppedNoteIdToPose.put(nextDroppedNoteId++, pose);
  }

  private static Map<Integer, Pose2d> droppedNoteIdToPose = new HashMap<>();

  private final int droppedNoteId;

  public AutoNoteDropped(int droppedNoteId) {
    this.droppedNoteId = droppedNoteId;
  }

  public Optional<Pose2d> getPose() {
    if (droppedNoteIdToPose.containsKey(droppedNoteId)) {
      return Optional.of(droppedNoteIdToPose.get(droppedNoteId));
    }

    return Optional.empty();
  }
}

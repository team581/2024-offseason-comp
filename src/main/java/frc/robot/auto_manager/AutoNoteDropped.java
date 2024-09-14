// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class AutoNoteDropped {
  private static int nextDroppedNoteId = 0;

  public static void clearDroppedNotes() {
    droppedNoteIdToPose.clear();
    nextDroppedNoteId = 0;
    DogLog.log("Debug/ClearedNoteNextId", nextDroppedNoteId);
    DogLog.log("Debug/ClearedNoteHashMap", droppedNoteIdToPose.toString());
    // log next dropped note id
    // log translation
    // log size of the map
  }

  public static void addDroppedNote(Translation2d translation) {

    DogLog.log("Debug/AddDroppedNoteBeforeAdding", nextDroppedNoteId);
    DogLog.log("Debug/HashMapBeforeAdding", droppedNoteIdToPose.toString());
    droppedNoteIdToPose.put(nextDroppedNoteId++, translation);
    DogLog.log("Debug/AddDroppedNoteAfterAdding", nextDroppedNoteId);
    DogLog.log("Debug/HashMapAfterAdding", droppedNoteIdToPose.toString());
    // log next dropped note id
    // log translation
    // log size of the map
  }

  private static Map<Integer, Translation2d> droppedNoteIdToPose = new HashMap<>();

  private final int droppedNoteId;

  public AutoNoteDropped(int droppedNoteId) {
    this.droppedNoteId = droppedNoteId;
  }

  public Optional<Translation2d> getPose() {
    // log stuff here also if its useful

    if (droppedNoteIdToPose.containsKey(droppedNoteId)) {
      return Optional.of(droppedNoteIdToPose.get(droppedNoteId));
    }

    return Optional.empty();
  }
}

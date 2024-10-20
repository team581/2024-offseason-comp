// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoNoteStep(AutoNoteAction action, List<Supplier<Optional<Translation2d>>> notes) {
  static AutoNoteStep cleanup() {
    return new AutoNoteStep(AutoNoteAction.CLEANUP, List.of());
  }

  public static AutoNoteStep drop(Integer... noteIds) {
    return new AutoNoteStep(AutoNoteAction.DROP, noteIdsToSearchPoseSuppliers(noteIds));
  }

  public static AutoNoteStep score(Integer... noteIds) {
    return new AutoNoteStep(AutoNoteAction.SCORE, noteIdsToSearchPoseSuppliers(noteIds));
  }

  private static Supplier<Optional<Translation2d>> noteIdToPose(int id) {
    DogLog.log("Debug/NoteIdToPose", id);
    if (id >= 10) {
      // Dropped note ID
      Supplier<Optional<Translation2d>> note = new AutoNoteDropped(id - 10)::getPose;
      if (note.get().isPresent()) {
        DogLog.log("Debug/NoteIDToPoseDropped", note.get().get());
      }
      return note;
    }

    return new AutoNoteStaged(id)::getPose;
  }

  private static List<Supplier<Optional<Translation2d>>> noteIdsToSearchPoseSuppliers(
      Integer... noteIds) {
    List<Integer> list = List.of(noteIds);

    return list.stream()
        .map(
            noteId -> {
              // Explicit about types here since Java can't infer the return types
              Supplier<Optional<Translation2d>> searchPoseSupplier =
                  () -> AutoNoteStep.noteIdToPose(noteId).get();

              return searchPoseSupplier;
            })
        .toList();
  }

  public AutoNoteStep(AutoNoteAction action, Supplier<Optional<Translation2d>>... noteSuppliers) {
    this(action, List.of(noteSuppliers));
  }
}

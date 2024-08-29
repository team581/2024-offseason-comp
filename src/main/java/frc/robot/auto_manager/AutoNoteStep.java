// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoNoteStep(AutoNoteAction action, List<Supplier<Optional<Translation2d>>> notes) {
  static AutoNoteStep cleanup() {
    return new AutoNoteStep(AutoNoteAction.CLEANUP, List.of());
  }

  public static AutoNoteStep drop(Integer... noteIds) {
    List<Integer> list = List.of(noteIds);

    return new AutoNoteStep(
        AutoNoteAction.DROP, list.stream().map(AutoNoteStep::noteIdToPose).toList());
  }

  public static AutoNoteStep dropPreload() {
    return new AutoNoteStep(AutoNoteAction.DROP_PRELOAD, List.of());
  }

  public static AutoNoteStep score(Integer... noteIds) {
    List<Integer> list = List.of(noteIds);

    return new AutoNoteStep(
        AutoNoteAction.SCORE, list.stream().map(AutoNoteStep::noteIdToPose).toList());
  }

  private static Supplier<Optional<Translation2d>> noteIdToPose(int id) {
    if (id >= 10) {
      // Dropped note ID
      return new AutoNoteDropped(id - 10)::getPose;
    }

    return new AutoNoteStaged(id)::getPose;
  }

  public AutoNoteStep(AutoNoteAction action, Supplier<Optional<Translation2d>>... notes) {
    this(action, List.of(notes));
  }
}

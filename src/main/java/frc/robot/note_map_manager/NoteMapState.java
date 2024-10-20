// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager;

public enum NoteMapState {
  STOPPED,
  WAITING_FOR_NOTES,
  INTAKING_PATHFINDING,
  INTAKING_PID,
  PATHFIND_TO_DROP,
  PATHFIND_TO_SCORE,
  DROP,
  SCORE,
  CLEANUP,
  SEARCH_MIDLINE_FIRST,
  SEARCH_MIDLINE_SECOND,
  SEARCH_SPEAKER_FIRST,
  SEARCH_SPEAKER_SECOND,
  SEARCH_SPEAKER_THIRD;
}

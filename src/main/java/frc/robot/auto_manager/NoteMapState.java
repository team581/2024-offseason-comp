// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

public enum NoteMapState {
  IDLE,
  PATHFIND_TO_INTAKE,
  PID_INTAKE,
  PATHFIND_TO_DROP,
  PATHFIND_TO_SCORE,
  DROP,
  SCORE,
  CLEANUP,
  SEARCH_MIDLINE,
  SEARCH_SPEAKER;
}

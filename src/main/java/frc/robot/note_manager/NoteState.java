// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_manager;

public enum NoteState {
  IDLE_NO_GP,

  /** Note is in intake, going to be sent to queuer for idle. */
  INTAKE_TO_QUEUER,
  INTAKE_TO_QUEUER_FOR_SHOOTING,
  LAZY_INTAKE_TO_QUEUER,

  IDLE_IN_QUEUER,
  GROUND_NOTE_TO_INTAKE,
  DROPPING,

  SHOOTING,
  SHOOTER_OUTTAKING,

  QUEUER_TO_CONVEYOR,
  QUEUER_TO_CONVEYOR_FINAL,
  IDLE_IN_CONVEYOR,

  CONVEYOR_TO_QUEUER,
  CONVEYOR_TO_QUEUER_FOR_OUTTAKING,
  CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKING,
  CONVEYOR_TO_QUEUER_FOR_SHOOTING,

  AMP_SCORING,
  TRAP_SCORING,

  UNJAM,

  QUEUER_TO_INTAKE_FOR_OUTTAKING,
  OUTTAKING;
}

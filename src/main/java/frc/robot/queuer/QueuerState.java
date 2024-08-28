// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

public enum QueuerState {
  IDLE,
  QUEUER_TO_CONVEYOR_FINAL,
  INTAKING,
  PASS_TO_SHOOTER,

  PASS_TO_INTAKE;
}

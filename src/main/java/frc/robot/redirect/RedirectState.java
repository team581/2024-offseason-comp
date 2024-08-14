// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.redirect;

public enum RedirectState {
  IDLE,
  INTAKE_TO_QUEUER,
  QUEUER_TO_CONVEYOR,
  TO_OUTTAKE,
  CONVEYOR_TO_QUEUER,
  QUEUER_TO_SHOOTER;
}

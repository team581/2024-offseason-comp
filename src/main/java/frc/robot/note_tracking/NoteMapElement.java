// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_tracking;

import edu.wpi.first.math.geometry.Translation2d;

public record NoteMapElement(double expiresAt, Translation2d noteTranslation, int health) {
  public NoteMapElement(double expiry, Translation2d translation) {
    this(expiry, translation, 5);
  }
}

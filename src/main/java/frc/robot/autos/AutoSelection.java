// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),
  OP_NON_NM("Red OP", "Blue OP"),
  AMP_SIDE_NM("Red Amp OP NM", "Blue Amp OP NM"),
  SOURCE_SIDE_NM("Red Source Side Race NM", "Blue Source Side Race NM");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}

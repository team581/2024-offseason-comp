// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),

  CHOREO_SIX_PIECE("Red Six Piece 2 to 4", ""),
  CHOREO_SNAKE_NT("Red Snake NT", ""),
  CHOREO_OP("Red OP", ""),
  CHOREO_SEVEN_PIECE("Red 7 Piece 2 to 6", ""),
  CHOREO_THREE_PIECE("Red 3 Piece 8 to 6", ""),
  CHOREO_FOUR_PIECE_2_3("Red 4 Piece 2 to 3", ""),
  CHOREO_1_3_NT("Red 1 to 3 + NT", ""),
  CHOREO_FOUR_PIECE_2_5("Red 4 Piece 2 to 5", ""),
  CHOREO_SOURCE_ALT("Red 3 Piece Alt", ""),
  CHOREO_NT_SOURCE("Red Source NT", ""),
  TEST_PATH("testPath", "");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}

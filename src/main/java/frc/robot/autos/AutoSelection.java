// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),

  // CHOREO_SNAKE_NT("Red Snake NT", "Blue Snake NT"),
  // CHOREO_SEVEN_PIECE("Red 7 Piece 2 to 6", "Blue 7 Piece 2 to 6"),
  //  CHOREO_1_3_NT("Red 1 to 3 + NT", "Blue 1 to 3 + NT"),
  // CHOREO_NT_SOURCE("Red Source NT", "Blue Source NT"),
  // CHOREO_SNAKE_AMP("Red Snake Amp", "Blue Snake Amp"),
  CHOREO_DROP_7_6("Red Drop 7 to 6", "Blue Drop 7 to 6"),
  // CHOREO_FOUR_PIECE_2_5("Red 4 Piece 2 to 5", "Blue 4 Piece 2 to 5"),
  // CHOREO_3_PIECE_8_6("Red 3 Piece 8 to 6", "Blue 3 Piece 8 to 6"),
  CHOREO_3_PIECE_8_7_DROP("Red 3 Piece 8 to 7 + 10", "Blue 3 Piece 8 to 7 + 10"),
  CHOREO_3_PIECE_DROP("Red 3 Piece 8 to 7 + 10", "Blue 3 Piece 8 to 7 + 10"),
  CHOREO_SIX_PIECE("Red Six Piece 2 to 4", "Blue Six Piece 2 to 4"),
  CHOREO_OP("Red OP", "Blue OP"),
  CHOREO_FOUR_PIECE_2_3("Red 4 Piece 2 to 3", "Blue 4 Piece 2 to 3"),
  CHOREO_3_PIECE_7_8_DROP("Red 3 Piece 7 to 8 + 10", "Blue 3 Piece 7 to 8 + 10"),
  CHOREO_AMP_DROP("Red Amp 3 Piece 7 to 8 + 10", "Blue Amp 3 Piece 7 to 8 + 10"),
  CHOREO_SOURCE_ALT("Red 3 Piece Alt", "Blue 3 Piece Alt");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}

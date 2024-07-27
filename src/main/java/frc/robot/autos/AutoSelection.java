// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),

  /// FOUR_PIECE_SOURCE_8_6_DYN("Red 4 Piece Source 8-6", "Blue 4 Piece Source 8-6"),
  /// OP_FIVE_PIECE("Red 5 Piece OP", "Blue 5 Piece OP"),
  /// OP_FIVE_PIECE_OFFSET("Red 5 Piece OP offset", "Blue 5 Piece OP offset"),
  /// SNAKE_FOUR_DYN("Red Right Snake, 4", "Blue Right Snake, 4"),
  /// SNAKE_FIVE_DYN("Red Right Snake, 5", "Blue Right Snake, 5"),
  /// SIX_PIECE_2_5_DYN("Red 6 Piece 2,1,3-5", "Blue 6 Piece 2,1,3-5"),
  /// SIX_PIECE_2_6_DYN("Red 6 Piece 2,1,3,5,6", "Blue 6 Piece 2,1,3,5,6"),
  /// BACKUP_SIX_PIECE_2_5("Backup Red 6 Piece", "Backup Blue 6 Piece"),
  CHOREO_SIX_PIECE("Red Six Piece 2 to 4.1", ""),
  CHOREO_AS_5_NT("Red AS to 6 + NT", ""),
  CHOREO_SNAKE_NT("Red Snake NT", ""),
  CHOREO_OP("Red OP", ""),
  CHOREO_SEVEN_PIECE("Red 7 Piece 2 to 6", ""),
  CHOREO_THREE_PIECE("Red 3 Piece 8 to 6", ""),
  CHOREO_FOUR_PIECE("Red 4 Piece 2 to 3", ""),
  TEST_PATH("testPath", "");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}

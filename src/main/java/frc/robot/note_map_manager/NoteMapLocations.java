// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import java.util.List;

public class NoteMapLocations {
  public static final Pose2d RED_AMP_SIDE_SCORING_LOCATION =
      new Pose2d(12.8, 5.9, Rotation2d.fromDegrees(-10.97));
  public static final Pose2d RED_SOURCE_SIDE_SCORING_LOCATION =
      new Pose2d(13.33, 2.83, Rotation2d.fromDegrees(37.59));

  public static final Pose2d BLUE_AMP_SIDE_SCORING_LOCATION =
      new Pose2d(3.77, 5.9, Rotation2d.fromDegrees(-168.48));
  public static final Pose2d BLUE_SOURCE_SIDE_SCORING_LOCATION =
      new Pose2d(3.01, 2.83, Rotation2d.fromDegrees(142.77));
  public static final List<Pose2d> RED_SCORING_LOCATIONS =
      List.of(
          // Amp Side
          RED_AMP_SIDE_SCORING_LOCATION,
          // Source side
          RED_SOURCE_SIDE_SCORING_LOCATION);

  public static final List<Pose2d> BLUE_SCORING_LOCATIONS =
      List.of(
          // Amp Side
          BLUE_AMP_SIDE_SCORING_LOCATION,
          // Source side
          BLUE_SOURCE_SIDE_SCORING_LOCATION);

  public static final List<Pose2d> RED_DROPPING_DESTINATIONS =
      List.of(
          new Pose2d(9.271, 7.458, Rotation2d.fromDegrees(0.0)),
          new Pose2d(9.271, 5.782, Rotation2d.fromDegrees(0.0)),
          new Pose2d(9.271, 4.106, Rotation2d.fromDegrees(0.0)),
          new Pose2d(9.271, 2.429, Rotation2d.fromDegrees(0.0)),
          new Pose2d(9.271, 0.753, Rotation2d.fromDegrees(0.0)));

  public static final List<Pose2d> RED_DROPPING_DESTINATIONS_HOME =
      List.of(
          new Pose2d(10.5, 7.458, Rotation2d.fromDegrees(0.0)),
          new Pose2d(10.5, 5.782, Rotation2d.fromDegrees(0.0)),
          new Pose2d(10.5, 4.106, Rotation2d.fromDegrees(0.0)),
          new Pose2d(10.5, 2.429, Rotation2d.fromDegrees(0.0)),
          new Pose2d(10.5, 0.753, Rotation2d.fromDegrees(0.0)));

  public static final List<Pose2d> BLUE_DROPPING_DESTINATIONS =
      List.of(
          new Pose2d(7.271, 7.458, Rotation2d.fromDegrees(180.0)),
          new Pose2d(7.271, 5.782, Rotation2d.fromDegrees(180.0)),
          new Pose2d(7.271, 4.106, Rotation2d.fromDegrees(180.0)),
          new Pose2d(7.271, 2.429, Rotation2d.fromDegrees(180.0)),
          new Pose2d(7.271, 0.753, Rotation2d.fromDegrees(180.0)));

  public static List<Pose2d> getScoringDestinations() {
    if (FmsSubsystem.isRedAlliance()) {
      return NoteMapLocations.RED_SCORING_LOCATIONS;
    } else {
      return NoteMapLocations.BLUE_SCORING_LOCATIONS;
    }
  }

  public static List<Pose2d> getDroppingDestinations() {
    if (FmsSubsystem.isRedAlliance()) {
      if (RobotConfig.IS_HOME) {
        return NoteMapLocations.RED_DROPPING_DESTINATIONS_HOME;
      }
      return NoteMapLocations.RED_DROPPING_DESTINATIONS;
    } else {
      return NoteMapLocations.BLUE_DROPPING_DESTINATIONS;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class NoteMapLocations {
  public static final List<Pose2d> RED_SPEAKER_CLEANUP_PATH =
      List.of(
          new Pose2d(12.67, 6.98, Rotation2d.fromDegrees(140.75)),
          new Pose2d(14.13, 6.10, Rotation2d.fromDegrees(-159.24)),
          new Pose2d(14.48, 4.09, Rotation2d.fromDegrees(174.67)));
  public static final List<Pose2d> BLUE_SPEAKER_CLEANUP_PATH =
      List.of(
          new Pose2d(3.62, 6.7, Rotation2d.fromDegrees(47.03)),
          new Pose2d(2.57, 6.34, Rotation2d.fromDegrees(-7.35)),
          new Pose2d(2.34, 4.49, Rotation2d.fromDegrees(16.93)));
  public static final Pose2d MIDLINE_CLEANUP_POSE = new Pose2d(8.271, 4.106, new Rotation2d(0));
  public static final List<Pose2d> RED_SCORING_DESTINATIONS =
      List.of(
          new Pose2d(11.82, 6.49, Rotation2d.fromDegrees(-11.04)),
          new Pose2d(13.36, 1.51, Rotation2d.fromDegrees(53.25)));

  public static final List<Pose2d> BLUE_SCORING_DESTINATIONS =
      List.of(
          new Pose2d(4.32, 6.41, Rotation2d.fromDegrees(-169.48)),
          new Pose2d(4.29, 5.02, Rotation2d.fromDegrees(172.97)),
          new Pose2d(3.17, 3.21, Rotation2d.fromDegrees(142.74)));

  public static final List<Pose2d> RED_DROPPING_DESTINATIONS =
      List.of(
          new Pose2d(11.39, 7.5, Rotation2d.fromDegrees(0.0)),
          new Pose2d(11.91, 4.59, Rotation2d.fromDegrees(0.0)),
          new Pose2d(11.38, 1.94, Rotation2d.fromDegrees(19.08)));

  public static final List<Pose2d> BLUE_DROPPING_DESTINATIONS =
      List.of(
          new Pose2d(5.21, 7.5, Rotation2d.fromDegrees(180)),
          new Pose2d(4.8, 4.65, Rotation2d.fromDegrees(180)),
          new Pose2d(5.25, 1.92, Rotation2d.fromDegrees(144.57)));

  public static final Pose2d RED_DROPPING_DESTINATION =
      new Pose2d(11.25, 7.26, Rotation2d.fromDegrees(16.18));
  public static final Pose2d BLUE_DROPPING_DESTINATION =
      new Pose2d(5.33, 7.26, Rotation2d.fromDegrees(167.95));

  public static final BoundingBox RED_SCORING_BOX =
      new BoundingBox(
          new Translation2d(11.12, 7.85),
          new Translation2d(15.4, 7.61),
          new Translation2d(11.75, 4.96),
          new Translation2d(15.78, 3.19));
  public static final BoundingBox BLUE_SCORING_BOX =
      new BoundingBox(
          new Translation2d(1.1, 7.57),
          new Translation2d(5.08, 7.72),
          new Translation2d(1.11, 3.49),
          new Translation2d(4.97, 4.67));

  public static final List<Pose2d> RED_MIDLINE_CLEANUP_PATH =
      List.of(
          new Pose2d(9.83, 2.42, Rotation2d.fromDegrees(43.18)),
          new Pose2d(9.83, 5.69, Rotation2d.fromDegrees(-41.89)));
  public static final List<Pose2d> BLUE_MIDLINE_CLEANUP_PATH =
      List.of(
          new Pose2d(6.66, 2.42, Rotation2d.fromDegrees(136.86)),
          new Pose2d(6.66, 5.69, Rotation2d.fromDegrees(-144.39)));
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_map.NoteMap;
import frc.robot.note_map.NoteMapElement;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class AutoManager extends LifecycleSubsystem {
  private final RobotCommands actions;
  private final NoteMap noteMap;
  private final RobotManager robotManager;
  private final LocalizationSubsystem localization;
  private static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(2.0, 2.0, 2 * Math.PI, 4 * Math.PI);

  public static final Pose2d RED_SPEAKER_CLEANUP_POSE =
      new Pose2d(
          Units.inchesToMeters(652.73) - 2.0,
          Units.inchesToMeters(218.42),
          Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_SPEAKER_CLEANUP_POSE =
      new Pose2d(
          Units.inchesToMeters(0) + 2.0, Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));
  public static final Pose2d MIDLINE_CLEANUP_POSE = new Pose2d(8.271, 4.106, new Rotation2d(0));

  public static final List<Pose2d> RED_DESTINATIONS =
      List.of(
          new Pose2d(12.32, 5.16, Rotation2d.fromDegrees(5.62)),
          new Pose2d(12.73, 6.00, Rotation2d.fromDegrees(-6.73)),
          new Pose2d(13.12, 7.13, Rotation2d.fromDegrees(-25.16)),
          new Pose2d(13.23, 5.56, Rotation2d.fromDegrees(0.0)),
          new Pose2d(14.28, 4.23, Rotation2d.fromDegrees(31.66)),
          new Pose2d(15.02, 5.52, Rotation2d.fromDegrees(0.0)),
          new Pose2d(15.35, 6.83, Rotation2d.fromDegrees(-58.15)),
          new Pose2d(15.57, 4.19, Rotation2d.fromDegrees(60)),
          new Pose2d(11.76, 6.56, Rotation2d.fromDegrees(-12.58)),
          new Pose2d(13.12, 3.12, Rotation2d.fromDegrees(36.39)),
          new Pose2d(12.92, 2.2, Rotation2d.fromDegrees(43.44)),
          new Pose2d(12.92, 2.2, Rotation2d.fromDegrees(43.44)),
          new Pose2d(14.66, 3.27, Rotation2d.fromDegrees(51.23)),
          new Pose2d(14.83, 7.42, Rotation2d.fromDegrees(-56.14)));

  public static final List<Pose2d> BLUE_DESTINATIONS =
      List.of(
          new Pose2d(4.09, 5.16, Rotation2d.fromDegrees(174.8)),
          new Pose2d(4.37, 6.23, Rotation2d.fromDegrees(-171.57)),
          new Pose2d(3.38, 7.13, Rotation2d.fromDegrees(-151.77)),
          new Pose2d(3.3, 5.49, Rotation2d.fromDegrees(-177.97)),
          new Pose2d(2.50, 4.19, Rotation2d.fromDegrees(149.16)),
          new Pose2d(3.09, 3.23, Rotation2d.fromDegrees(140.93)),
          new Pose2d(0.85, 4.1, Rotation2d.fromDegrees(113.51)),
          new Pose2d(1.04, 3.12, Rotation2d.fromDegrees(109.05)),
          new Pose2d(1.6, 5.49, Rotation2d.fromDegrees(180.0)),
          new Pose2d(2.14, 5.51, Rotation2d.fromDegrees(180.05)),
          new Pose2d(0.91, 6.8, Rotation2d.fromDegrees(-124.38)),
          new Pose2d(1.86, 7.02, Rotation2d.fromDegrees(-140.23)),
          new Pose2d(2.35, 6.25, Rotation2d.fromDegrees(-162.2)),
          new Pose2d(4.6, 6.83, Rotation2d.fromDegrees(-162.20)),
          new Pose2d(3.57, 2.72, Rotation2d.fromDegrees(138.0)));

  public AutoManager(
      RobotCommands actions,
      NoteMap noteMap,
      RobotManager robotManager,
      LocalizationSubsystem localization) {
    super(SubsystemPriority.AUTOS);
    this.actions = actions;
    this.noteMap = noteMap;
    this.robotManager = robotManager;
    this.localization = localization;
  }

  private List<Pose2d> getScoringDestinations() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_DESTINATIONS;
    } else {
      return BLUE_DESTINATIONS;
    }
  }

  private Pose2d getClosestScoringDestination() {
    Pose2d current = localization.getPose();

    Pose2d closest = getScoringDestinations().get(0);
    double currentDistance = Double.POSITIVE_INFINITY;

    for (Pose2d target : getScoringDestinations()) {
      double distance = target.getTranslation().getDistance(current.getTranslation());
      if (distance < currentDistance) {
        closest = target;
        currentDistance = distance;
      }
    }

    return closest;
  }

  private static Pose2d getSpeakerCleanupPose() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_SPEAKER_CLEANUP_POSE;
    } else {
      return BLUE_SPEAKER_CLEANUP_POSE;
    }
  }

  public Command doManyAutoSteps(List<AutoNoteStep> steps) {
    DogLog.log("Debug/DoManyAutoSteps", true);

    return Commands.sequence(steps.stream().map(this::doAutoStep).toArray(Command[]::new));
  }

  private Command pathfindToScore() {
    return Commands.defer(
            () -> {
              return AutoBuilder.pathfindToPose(
                  getClosestScoringDestination(), DEFAULT_CONSTRAINTS);
            },
            Set.of())
        .raceWith(actions.waitForSpeakerShotCommand())
        .andThen(actions.speakerShotCommand())
        .onlyIf(() -> robotManager.getState().hasNote);
  }

  private Command pathfindToOuttake() {
    return Commands.defer(
            () -> {
              return AutoBuilder.pathfindToPose(
                  getClosestScoringDestination(), DEFAULT_CONSTRAINTS);
            },
            Set.of())
        .andThen(actions.dropCommand())
        .onlyIf(() -> robotManager.getState().hasNote);
  }

  private Command cleanupNote() {
    // find and score a note
    DogLog.log("Debug/CleanupNote", true);

    return noteMap.intakeNearestMapNote(3.0).andThen(pathfindToScore());
  }

  public Command cleanupCommand() {
    var robotPose = localization.getPose();
    var speakerCleanupPose = getSpeakerCleanupPose();
    // if we're close to speaker
    if (robotPose.getTranslation().getDistance(speakerCleanupPose.getTranslation()) < 3.0) {
      DogLog.log("Debug/SpeakerCleanup", true);

      return AutoBuilder.pathfindToPose(speakerCleanupPose, DEFAULT_CONSTRAINTS)
          .until(() -> noteMap.mapContainsNote(robotPose, 3.0))
          .andThen(
              cleanupNote()
                  .repeatedly()
                  .onlyWhile(() -> noteMap.mapContainsNote() || robotManager.getState().hasNote));
    }

    // if we're close to midline
    DogLog.log("Debug/MidlineCleanup", true);
    return AutoBuilder.pathfindToPose(MIDLINE_CLEANUP_POSE, DEFAULT_CONSTRAINTS)
        .until(() -> noteMap.mapContainsNote(robotPose, 2.0))
        .andThen(
            cleanupNote()
                .repeatedly()
                .onlyWhile(() -> noteMap.mapContainsNote() || robotManager.getState().hasNote));
  }

  private Command doAutoStep(AutoNoteStep step) {
    if (step.action() == AutoNoteAction.CLEANUP) {
      return cleanupCommand();
    }

    var command =
        noteMap.intakeNoteAtPose(
            () -> {
              DogLog.log("Debug/IntakeNoteAtPoseRequest", true);
              return step.noteSearchPose().get();
            },
            1.5);

    if (step.action() == AutoNoteAction.OUTTAKE) {
      command = command.andThen(pathfindToOuttake());
    } else if (step.action() == AutoNoteAction.SCORE) {
      command = command.andThen(pathfindToScore());
    }

    return command;
  }

  public Command testCommand() {
    DogLog.log("Debug/TestCommand", true);

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              DogLog.log("Debug/ResetNoteMap", true);

              var now = Timer.getFPGATimestamp();
              noteMap.resetNoteMap(
                  new ArrayList<>(
                      List.of(
                          new NoteMapElement(now + 5, AutoNoteStep.noteIdToPose(4)),
                          new NoteMapElement(now + 5, AutoNoteStep.noteIdToPose(5)),
                          new NoteMapElement(now + 5, AutoNoteStep.noteIdToPose(3)))));
            }),
        doManyAutoSteps(
            List.of(
                new AutoNoteStep(3, AutoNoteAction.OUTTAKE),
                new AutoNoteStep(4, AutoNoteAction.SCORE),
                new AutoNoteStep(5, AutoNoteAction.CLEANUP))));
  }
}

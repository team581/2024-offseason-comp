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
import frc.robot.note_tracking.NoteMapElement;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoManager extends LifecycleSubsystem {
  private final RobotCommands actions;
  private final NoteTrackingManager noteTrackingManager;
  private final RobotManager robotManager;
  private final LocalizationSubsystem localization;
  private static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(5.0, 5.0, 2 * Math.PI, 4 * Math.PI);

  public static final Pose2d RED_SPEAKER_CLEANUP_POSE =
      new Pose2d(
          Units.inchesToMeters(652.73) - 2.0,
          Units.inchesToMeters(218.42),
          Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_SPEAKER_CLEANUP_POSE =
      new Pose2d(
          Units.inchesToMeters(0) + 2.0, Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));
  public static final Pose2d MIDLINE_CLEANUP_POSE = new Pose2d(8.271, 4.106, new Rotation2d(0));
  public static final List<Pose2d> RED_SCORING_DESTINATIONS =
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

  public static final List<Pose2d> BLUE_SCORING_DESTINATIONS =
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

  public static final Pose2d RED_DROPPING_DESTINATION =
      new Pose2d(11.25, 7.26, Rotation2d.fromDegrees(16.18));
  public static final Pose2d BLUE_DROPPING_DESTINATION =
      new Pose2d(5.33, 7.26, Rotation2d.fromDegrees(167.95));
  private Pose2d DROPPED_NOTE_SEARCH = new Pose2d(13.49, 7.40, Rotation2d.fromDegrees(170.5));

  public AutoManager(
      RobotCommands actions,
      NoteTrackingManager noteTrackingManager,
      RobotManager robotManager,
      LocalizationSubsystem localization) {
    super(SubsystemPriority.AUTOS);
    this.actions = actions;
    this.noteTrackingManager = noteTrackingManager;
    this.robotManager = robotManager;
    this.localization = localization;
  }

  private List<Pose2d> getScoringDestinations() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_SCORING_DESTINATIONS;
    } else {
      return BLUE_SCORING_DESTINATIONS;
    }
  }

  private Pose2d getDroppingDestination() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_DROPPING_DESTINATION;
    } else {
      return BLUE_DROPPING_DESTINATION;
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
    DogLog.log("Debug/DoManyAutoSteps", Timer.getFPGATimestamp());

    return Commands.sequence(steps.stream().map(this::doAutoStep).toArray(Command[]::new));
  }

  private Command cleanupNote() {
    // find and score a note
    DogLog.log("Debug/CleanupNote", Timer.getFPGATimestamp());

    return noteTrackingManager
        .intakeNearestMapNote(2.0)
        .andThen(
            Commands.deferredProxy(
                    () -> {
                      DogLog.log("Debug/CleanupNotePathfind", Timer.getFPGATimestamp());
                      return AutoBuilder.pathfindToPose(
                          getClosestScoringDestination(), DEFAULT_CONSTRAINTS);
                    })
                .andThen(actions.speakerShotCommand())
                .unless(() -> !robotManager.getState().hasNote));
  }

  private Command cleanupCommand() {
    var robotPose = localization.getPose();
    var speakerCleanupPose = getSpeakerCleanupPose();
    // if we're close to speaker
    if (robotPose.getTranslation().getDistance(speakerCleanupPose.getTranslation()) < 3.0) {
      DogLog.log("Debug/SpeakerCleanup", Timer.getFPGATimestamp());

      return AutoBuilder.pathfindToPose(speakerCleanupPose, DEFAULT_CONSTRAINTS)
          .until(noteTrackingManager::mapContainsNote)
          .andThen(cleanupNote().repeatedly().onlyWhile(noteTrackingManager::mapContainsNote));
    }

    // if we're close to midline
    DogLog.log("Debug/MidlineCleanup", Timer.getFPGATimestamp());

    return AutoBuilder.pathfindToPose(MIDLINE_CLEANUP_POSE, DEFAULT_CONSTRAINTS)
        .until(noteTrackingManager::mapContainsNote)
        .andThen(cleanupNote().repeatedly().onlyWhile(noteTrackingManager::mapContainsNote));
  }

  private Command scoreCommand() {
    return Commands.deferredProxy(
            () -> {
              DogLog.log("Debug/PathFindShoot", Timer.getFPGATimestamp());
              return AutoBuilder.pathfindToPose(
                  getClosestScoringDestination(), DEFAULT_CONSTRAINTS);
            })
        .andThen(actions.speakerShotCommand())
        .unless(
            () -> {
              var shouldNotRun = !robotManager.getState().hasNote;

              DogLog.log("Debug/ScoreShouldNotRun", shouldNotRun);

              return shouldNotRun;
            });
  }

  private Command dropCommand() {
    return Commands.sequence(
            // Pathfind to outtake
            Commands.deferredProxy(
                () -> {
                  DogLog.log("Debug/PathFindOuttake", Timer.getFPGATimestamp());
                  return AutoBuilder.pathfindToPose(getDroppingDestination(), DEFAULT_CONSTRAINTS);
                }),
            // Drop the note
            actions
                .dropCommand()
                .andThen(
                    Commands.runOnce(
                        () -> {
                          noteTrackingManager.addNoteToMap(DROPPED_NOTE_SEARCH);
                          AutoNoteDropped.addDroppedNote(DROPPED_NOTE_SEARCH);
                        })))
        .unless(
            () -> {
              var shouldNotRun = !robotManager.getState().hasNote;

              DogLog.log("Debug/DropShouldNotRun", shouldNotRun);

              return shouldNotRun;
            });
  }

  private Command doAutoStep(AutoNoteStep step) {
    return switch (step.action()) {
      case CLEANUP -> cleanupCommand();
      case DROP ->
          Commands.deferredProxy(
                  () -> {
                    for (var poseSupplier : step.notes()) {
                      Optional<Pose2d> maybePose = poseSupplier.get();

                      if (maybePose.isEmpty()) {
                        continue;
                      }

                      var pose = maybePose.get();

                      if (noteTrackingManager.getNearestNotePoseRelative(pose, 1.5).isPresent()) {
                        noteTrackingManager.intakeNoteAtPose(
                            () -> {
                              DogLog.log("Debug/IntakeNoteAtPoseRequest", Timer.getFPGATimestamp());
                              return pose;
                            },
                            1.5);
                      }
                    }

                    // None of the notes are tracked, so just entirely skip this step
                    return Commands.none();
                  })
              .until(() -> robotManager.getState().hasNote)
              .andThen(dropCommand());
      case SCORE ->
          Commands.deferredProxy(
                  () -> {
                    for (var poseSupplier : step.notes()) {
                      Optional<Pose2d> maybePose = poseSupplier.get();

                      if (maybePose.isEmpty()) {
                        continue;
                      }

                      var pose = maybePose.get();

                      if (noteTrackingManager.getNearestNotePoseRelative(pose, 1.5).isPresent()) {

                        // TODO: Find and score command needs to be restructured.
                        // We need to make it where it will attempt intaking note 4
                        // Then, it will intake note 5 (or next note idk)
                        // Just keep intaking, and the way we go to scoring/dropping will be a
                        // .until()
                        // so: Commands.sequence(intake(4), intake(5)).until(() -> robotHasNote() ==
                        // true).andThen(score).unless(no note)
                        // Intake note 4, then 5, etc.
                        // Cancel that process, once we have a note
                        // This prevents the evil logic of intaking multiple notes back to back
                        // Then score, unless we didn't get any notes :(

                        noteTrackingManager.intakeNoteAtPose(
                            () -> {
                              DogLog.log("Debug/IntakeNoteAtPoseRequest", Timer.getFPGATimestamp());
                              return pose;
                            },
                            1.5);
                      }
                    }

                    // None of the notes are tracked, so just entirely skip this step
                    return Commands.none();
                  })
              .until(() -> robotManager.getState().hasNote)
              .andThen(scoreCommand());
    };
  }

  public Command testCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              DogLog.log("Debug/TestP1", true);

              var now = Timer.getFPGATimestamp();
              noteTrackingManager.resetNoteMap(
                  new ArrayList<>(
                      List.of(
                          new NoteMapElement(now + 5, AutoNoteStaged.noteIdToPose(4)),
                          new NoteMapElement(now + 5, AutoNoteStaged.noteIdToPose(5)),
                          new NoteMapElement(now + 5, AutoNoteStaged.noteIdToPose(6)))));
            }),
        doManyAutoSteps(List.of(AutoNoteStep.score(4, 5), AutoNoteStep.score(5, 6))));
  }
}

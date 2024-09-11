// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import java.util.Set;
import java.util.function.Supplier;

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
  public static final List<Pose2d> RED_DESTINATIONS =
      List.of(
          new Pose2d(12.46, 6.35, Rotation2d.fromDegrees(-11.05)),
          new Pose2d(12.19, 4.98, Rotation2d.fromDegrees(8.28)),
          new Pose2d(13.67, 3.31, Rotation2d.fromDegrees(38.97)),
          new Pose2d(13.65, 5.553, new Rotation2d(0)));

  public static final List<Pose2d> BLUE_DESTINATIONS =
      List.of(
          new Pose2d(4.32, 6.41, Rotation2d.fromDegrees(-169.48)),
          new Pose2d(4.29, 5.02, Rotation2d.fromDegrees(172.97)),
          new Pose2d(3.17, 3.21, Rotation2d.fromDegrees(142.74)));

  public static final Pose2d RED_DROPPING_DESTINATION =
      new Pose2d(11.25, 7.26, Rotation2d.fromDegrees(16.18));
  public static final Pose2d BLUE_DROPPING_DESTINATION =
      new Pose2d(5.33, 7.26, Rotation2d.fromDegrees(167.95));
  private static final Translation2d DROPPED_NOTE_SEARCH = new Translation2d(13.49, 7.40);

  private static final BoundingBox RED_SCORING_BOX =
      new BoundingBox(
          new Translation2d(11.12, 7.85),
          new Translation2d(15.4, 7.61),
          new Translation2d(11.75, 4.96),
          new Translation2d(15.78, 3.19));
  private static final BoundingBox BLUE_SCORING_BOX =
      new BoundingBox(
          new Translation2d(1.1, 7.57),
          new Translation2d(5.08, 7.72),
          new Translation2d(1.11, 3.49),
          new Translation2d(4.97, 4.67));

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

  private BoundingBox getScoringBox() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_SCORING_BOX;
    } else {
      return BLUE_SCORING_BOX;
    }
  }

  private List<Pose2d> getScoringDestinations() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_DESTINATIONS;
    } else {
      return BLUE_DESTINATIONS;
    }
  }

  private Pose2d getDroppingDestination() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_DROPPING_DESTINATION;
    } else {
      return BLUE_DROPPING_DESTINATION;
    }
  }

  private static Pose2d getSpeakerCleanupPose() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_SPEAKER_CLEANUP_POSE;
    } else {
      return BLUE_SPEAKER_CLEANUP_POSE;
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

  private boolean robotInBox() {
    return getScoringBox().contains(localization.getPose().getTranslation());
  }

  private Command intakeNoteAtPose(Supplier<Optional<Translation2d>> pose) {
    Optional<Translation2d> maybeSearchArea = pose.get();
    if (maybeSearchArea.isPresent()) {
      return noteTrackingManager.intakeNoteAtPose(maybeSearchArea::get, 1.5);
    }
    return Commands.none();
  }

  private Command intakeAnyStepNotes(AutoNoteStep step) {
    // For each note in the step, try intaking until we get a note
    return Commands.sequence(
            step.notes().stream().map(this::intakeNoteAtPose).toArray(Command[]::new))
        .until(() -> robotManager.getState().hasNote);
  }

  private Command cleanupNote() {
    return noteTrackingManager.intakeNearestMapNote(10.0).andThen(pathfindToScoreCommand());
  }

  private Command cleanupCommand() {
    return cleanupNote().repeatedly();
  }

  public Command dropNote() {
    return actions
        .dropCommand()
        .andThen(
            Commands.runOnce(
                () -> {
                  var robotTranslation = localization.getPose().getTranslation();
                  noteTrackingManager.addNoteToMap(robotTranslation);
                  AutoNoteDropped.addDroppedNote(robotTranslation);
                }));
  }

  private Command pathfindToDropCommand() {
    return Commands.sequence(
            // Pathfind to outtake
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(getDroppingDestination(), DEFAULT_CONSTRAINTS),
                Set.of(robotManager.swerve)),
            // Drop the note
            dropNote())
        .onlyIf(() -> robotManager.getState().hasNote)
        .withTimeout(2.5);
  }

  private Command pathfindToScoreCommand() {
    return Commands.defer(
            () -> AutoBuilder.pathfindToPose(getClosestScoringDestination(), DEFAULT_CONSTRAINTS),
            Set.of(robotManager.swerve))
        .withTimeout(3)
        .andThen(actions.speakerShotCommand().until(() -> !robotManager.getState().hasNote))
        .onlyIf(
            () -> {
              boolean shouldScore = robotManager.getState().hasNote;
              DogLog.log("AutoManager/ShouldScore", shouldScore);
              return shouldScore;
            });
  }

  public Command doAutoStep(AutoNoteStep step) {
    return switch (step.action()) {
      case CLEANUP -> cleanupCommand();
      case DROP ->
          intakeAnyStepNotes(step)
              // Then, once we have a note, do the drop
              .andThen(pathfindToDropCommand())
              .withTimeout(6);
      case SCORE ->
          intakeAnyStepNotes(step)
              // Then, once we have a note, do the score
              .andThen(pathfindToScoreCommand())
              .withTimeout(6);
    };
  }

  public Command doManyAutoSteps(List<AutoNoteStep> steps) {
    return Commands.sequence(steps.stream().map(this::doAutoStep).toArray(Command[]::new));
  }

  public Command testCommand1() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              var now = Timer.getFPGATimestamp();
              noteTrackingManager.resetNoteMap(
                  new ArrayList<>(
                      List.of(
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(2)),
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(3)),
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(4)),
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(5)))));
            }),
        doManyAutoSteps(
            List.of(AutoNoteStep.score(2, 3), AutoNoteStep.score(3, 4), AutoNoteStep.score(4, 5))));
  }

  public Command testCommand() {
    return Commands.sequence(doManyAutoSteps(List.of(AutoNoteStep.cleanup())));
  }
}

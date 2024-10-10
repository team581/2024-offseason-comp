// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_tracking.NoteMapElement;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

public class AutoManager extends StateMachine<NoteMapState> {
  private final RobotCommands actions;
  private final NoteTrackingManager noteTrackingManager;
  private final RobotManager robotManager;
  private final LocalizationSubsystem localization;
  private static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(5.0, 5.0, 2 * Math.PI, 4 * Math.PI);
  private static final double TARGET_NOTE_THRESHOLD_METERS = 2.0;
  private static final double INTAKE_PATHFIND_THRESHOLD_METERS = 2.0;

  public AutoManager(
      RobotCommands actions,
      NoteTrackingManager noteTrackingManager,
      RobotManager robotManager,
      LocalizationSubsystem localization) {
    super(SubsystemPriority.AUTOS, NoteMapState.STOPPED);
    this.actions = actions;
    this.noteTrackingManager = noteTrackingManager;
    this.robotManager = robotManager;
    this.localization = localization;
  }

  private static BoundingBox getScoringBox() {
    if (FmsSubsystem.isRedAlliance()) {
      return NoteMapLocations.RED_SCORING_BOX;
    } else {
      return NoteMapLocations.BLUE_SCORING_BOX;
    }
  }

  private static List<Pose2d> getScoringDestinations() {
    if (FmsSubsystem.isRedAlliance()) {
      return NoteMapLocations.RED_SCORING_DESTINATIONS;
    } else {
      return NoteMapLocations.BLUE_SCORING_DESTINATIONS;
    }
  }

  private static List<Pose2d> getDroppingDestinations() {
    if (FmsSubsystem.isRedAlliance()) {
      return NoteMapLocations.RED_DROPPING_DESTINATIONS;
    } else {
      return NoteMapLocations.BLUE_DROPPING_DESTINATIONS;
    }
  }

  private static List<Pose2d> getSpeakerCleanupPath() {
    if (FmsSubsystem.isRedAlliance()) {
      return NoteMapLocations.RED_SPEAKER_CLEANUP_PATH;
    } else {
      return NoteMapLocations.BLUE_SPEAKER_CLEANUP_PATH;
    }
  }

  private static List<Pose2d> getMidlineCleanupPath() {
    if (FmsSubsystem.isRedAlliance()) {
      return NoteMapLocations.RED_MIDLINE_CLEANUP_PATH;
    } else {
      return NoteMapLocations.BLUE_MIDLINE_CLEANUP_PATH;
    }
  }

  private static Pose2d getClosestSpeaker() {
    if (FmsSubsystem.isRedAlliance()) {
      return VisionSubsystem.ORIGINAL_RED_SPEAKER;
    } else {
      return VisionSubsystem.ORIGINAL_BLUE_SPEAKER;
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

  private Pose2d getClosestDroppingDestination() {
    Pose2d current = localization.getPose();

    Pose2d closest = getDroppingDestinations().get(0);
    double currentDistance = Double.POSITIVE_INFINITY;

    for (Pose2d target : getDroppingDestinations()) {
      double distance = target.getTranslation().getDistance(current.getTranslation());
      if (distance < currentDistance) {
        closest = target;
        currentDistance = distance;
      }
    }

    return closest;
  }

  public Command testCommand() {
    return Commands.runOnce(
        () -> {
          var now = Timer.getFPGATimestamp();
          DogLog.log("AutoManager/TestCommandRun", Timer.getFPGATimestamp());
          noteTrackingManager.resetNoteMap(
              new ArrayList<>(
                  List.of(
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(4)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(5)))));
          var steps = new LinkedList<AutoNoteStep>();
          steps.add(AutoNoteStep.score(4));
          steps.add(AutoNoteStep.score(5));

          setSteps(steps);
        });
  }

  private Optional<Pose2d> maybeNotePose = Optional.empty();

  private Queue<AutoNoteStep> steps = new LinkedList<>();
  private Optional<AutoNoteStep> currentStep = Optional.empty();

  private Pose2d closestScoringLocation = new Pose2d();
  private Pose2d droppingDestination = new Pose2d();

  private Command noteMapCommand = Commands.none();

  public void setSteps(LinkedList<AutoNoteStep> newSteps) {
    steps = newSteps;
    setStateFromRequest(NoteMapState.WAITING_FOR_NOTES);
  }

  public void off() {
    currentStep = Optional.empty();
    steps = new LinkedList<>();
    maybeNotePose = Optional.empty();
    setStateFromRequest(NoteMapState.STOPPED);
  }

  @Override
  protected void collectInputs() {
    if (currentStep.isPresent()) {

      for (var maybeSearchPoseSupplier : currentStep.get().notes()) {
        var maybeSearchPose = maybeSearchPoseSupplier.get();
        if (maybeSearchPose.isEmpty()) {
          continue;
        }

        var rawSearchLocation = maybeSearchPose.get();
        var maybeCurrentTargetedNote =
            noteTrackingManager.getNoteNearPose(rawSearchLocation, TARGET_NOTE_THRESHOLD_METERS);

        if (maybeCurrentTargetedNote.isPresent()) {
          var noteDistanceAngle =
              VisionSubsystem.distanceAngleToTarget(
                  new Pose2d(maybeCurrentTargetedNote.get().noteTranslation(), new Rotation2d()),
                  localization.getPose());

          maybeNotePose =
              Optional.of(
                  new Pose2d(
                      maybeCurrentTargetedNote.get().noteTranslation(),
                      Rotation2d.fromDegrees(noteDistanceAngle.targetAngle() + 180)));

          if (maybeNotePose.isPresent()) {
            DogLog.log("AutoManager/MaybeNotePose", maybeNotePose.get());
          }
          break;
        } else {
          maybeNotePose = Optional.empty();
          continue;
        }
      }
    }
  }

  // State actions
  @Override
  protected void afterTransition(NoteMapState newState) {
    switch (newState) {
      case STOPPED -> {
        noteMapCommand.cancel();
        robotManager.stowRequest();
      }
      case WAITING_FOR_NOTES -> {
        noteMapCommand.cancel();
        DogLog.log("AutoManager/IdleAction", Timer.getFPGATimestamp());
        if (steps.isEmpty()) {
          // Nothing new to do, keep idling
          DogLog.log("AutoManager/IdleStepsEmpty", Timer.getFPGATimestamp());

        } else {
          DogLog.log("AutoManager/IdleStepsPresent", Timer.getFPGATimestamp());
          currentStep = Optional.of(steps.poll());
          if (currentStep.isEmpty()) {
            DogLog.log("AutoManager/IdleCurrentStepEmpty", Timer.getFPGATimestamp());
          } else {
            DogLog.log("AutoManager/IdleCurrentStepPresent", Timer.getFPGATimestamp());
          }
        }
      }
      case DROP -> {
        noteMapCommand.cancel();
        robotManager.dropRequest();
      }
      case SCORE -> {
        noteMapCommand.cancel();
        robotManager.speakerShotRequest();
      }
      case INTAKING_PATHFINDING -> {
        noteMapCommand.cancel();

        if (currentStep.isEmpty()) {
          DogLog.log("AutoManager/InPathActionCurrentStepEmpty", Timer.getFPGATimestamp());
          break;
        }

        if (maybeNotePose.isPresent()) {
          DogLog.log("AutoManager/InPathActionNoteExists", Timer.getFPGATimestamp());
          noteMapCommand =
              AutoBuilder.pathfindToPose(maybeNotePose.get(), DEFAULT_CONSTRAINTS)
                  .withName("PathfindIntake");
          noteMapCommand.schedule();
        } else {
          DogLog.log("AutoManager/InPathActionNoNoteExists", Timer.getFPGATimestamp());
        }
      }
      case INTAKING_PID -> {
        noteMapCommand.cancel();

        if (currentStep.isEmpty()) {
          DogLog.log("AutoManager/InPIDActionCurrentStepEmpty", Timer.getFPGATimestamp());
          break;
        }

        if (maybeNotePose.isPresent()) {
          DogLog.log("AutoManager/InPIDActionNoteExists", Timer.getFPGATimestamp());
          noteMapCommand =
              noteTrackingManager
                  .intakeNoteAtPose(
                      maybeNotePose.get().getTranslation(), TARGET_NOTE_THRESHOLD_METERS)
                  .withName("PIDIntake");
          noteMapCommand.schedule();
        }
      }
      case PATHFIND_TO_DROP -> {
        noteMapCommand.cancel();
        droppingDestination = getClosestDroppingDestination();
        noteMapCommand =
            AutoBuilder.pathfindToPose(droppingDestination, DEFAULT_CONSTRAINTS)
                .withName("PathfindDrop");
        noteMapCommand.schedule();
      }
      case PATHFIND_TO_SCORE -> {
        noteMapCommand.cancel();
        closestScoringLocation = getClosestScoringDestination();
        noteMapCommand =
            AutoBuilder.pathfindToPose(closestScoringLocation, DEFAULT_CONSTRAINTS)
                .withName("PathfindScore");
        noteMapCommand.schedule();
        robotManager.waitSpeakerShotRequest();
      }
      case CLEANUP -> {}
      case SEARCH_MIDLINE -> {}
      case SEARCH_SPEAKER -> {}
    }
  }

  // State transitions
  @Override
  protected NoteMapState getNextState(NoteMapState currentState) {
    return switch (currentState) {
      case STOPPED -> {
        yield currentState;
      }
      case WAITING_FOR_NOTES -> {
        if (currentStep.isPresent() && currentStep.get().action() == AutoNoteAction.CLEANUP) {
          DogLog.log("AutoManager/IdleToCleanup", Timer.getFPGATimestamp());
          yield NoteMapState.CLEANUP;
        } else if (currentStep.isPresent() && maybeNotePose.isPresent()) {
          DogLog.log("AutoManager/IdleToIntakePathfinding", Timer.getFPGATimestamp());

          yield NoteMapState.INTAKING_PATHFINDING;
        }

        DogLog.log("AutoManager/IdleToIdle", Timer.getFPGATimestamp());
        yield currentState;
      }
      case INTAKING_PATHFINDING -> {
        // If current step is empty
        if (currentStep.isEmpty()) {
          DogLog.log("AutoManager/IntakingPathfindCurrentStepEmpty", Timer.getFPGATimestamp());

          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If note doesn't exist on map
        if (maybeNotePose.isEmpty()) {
          DogLog.log("AutoManager/IntakingPathfindMapNoteGone", Timer.getFPGATimestamp());
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If we already have note go and score/drop
        if (robotManager.getState().hasNote) {
          DogLog.log("AutoManager/IntakingPathfindRobotHasNote", Timer.getFPGATimestamp());
          if (currentStep.isPresent() && currentStep.get().action() == AutoNoteAction.DROP) {
            yield NoteMapState.PATHFIND_TO_DROP;
          }
          yield NoteMapState.PATHFIND_TO_SCORE;
        }

        // If the note is still there and we are close enough go to PID intake mode
        if (maybeNotePose.isPresent()
            && maybeNotePose
                    .get()
                    .getTranslation()
                    .getDistance(localization.getPose().getTranslation())
                < INTAKE_PATHFIND_THRESHOLD_METERS) {
          DogLog.log("AutoManager/CloseEnoughStopPathfinding", Timer.getFPGATimestamp());

          yield NoteMapState.INTAKING_PID;
        }

        yield currentState;
      }
      case INTAKING_PID -> {
        // If current step is empty
        if (currentStep.isEmpty()) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }
        // If we already have note go and score/drop
        if (robotManager.getState().hasNote) {
          if (currentStep.isPresent() && currentStep.get().action() == AutoNoteAction.DROP) {
            yield NoteMapState.PATHFIND_TO_DROP;
          }
          yield NoteMapState.PATHFIND_TO_SCORE;
        }

        // If note doesn't exist on map
        if (maybeNotePose.isEmpty()) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        yield currentState;
      }
      case PATHFIND_TO_DROP -> {
        // If robot doesn't have a note, give up (go to next step)
        if (!robotManager.getState().hasNote) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // if we finished pathfinding, drop note
        if (localization.atTranslation(droppingDestination.getTranslation(), 0.2)) {
          yield NoteMapState.DROP;
        }
        yield currentState;
      }
      case PATHFIND_TO_SCORE -> {
        // If robot doesn't have a note, give up (go to next step)
        if (!robotManager.getState().hasNote) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If we're already at location to score, score the note
        if (localization.atTranslation(closestScoringLocation.getTranslation(), 0.2)) {
          yield NoteMapState.SCORE;
        }
        yield currentState;
      }
      case DROP -> robotManager.getState().hasNote ? currentState : NoteMapState.WAITING_FOR_NOTES;
      case SCORE -> {
        if (currentStep.isPresent() && currentStep.get().action().equals(AutoNoteAction.CLEANUP)) {
          if (robotManager.getState().hasNote) {
            yield currentState;
          }

          yield NoteMapState.CLEANUP;
        }
        if (robotManager.getState().hasNote) {
          yield currentState;
        }

        yield NoteMapState.WAITING_FOR_NOTES;
      }
      case CLEANUP ->
          localization.getPose().getTranslation().getDistance(getClosestSpeaker().getTranslation())
                  < 4.0
              ? NoteMapState.SEARCH_SPEAKER
              : NoteMapState.SEARCH_MIDLINE;
      case SEARCH_MIDLINE -> {
        if (noteTrackingManager.mapContainsNote() && !robotManager.getState().hasNote) {
          yield NoteMapState.INTAKING_PATHFINDING;
        }
        if (robotManager.getState().hasNote) {
          yield NoteMapState.PATHFIND_TO_SCORE;
        }
        if (localization.atTranslation(getMidlineCleanupPath().get(1).getTranslation(), 0.2)) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }
        yield currentState;
      }
      case SEARCH_SPEAKER -> {
        if (noteTrackingManager.mapContainsNote() && !robotManager.getState().hasNote) {
          yield NoteMapState.INTAKING_PATHFINDING;
        }
        if (robotManager.getState().hasNote) {
          yield NoteMapState.PATHFIND_TO_SCORE;
        }
        if (localization.atTranslation(getMidlineCleanupPath().get(1).getTranslation(), 0.2)) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }
        yield currentState;
      }
    };
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_map_manager.pathfinding.HeuristicPathFollowing;
import frc.robot.note_tracking.NoteMapElement;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.snaps.SnapManager;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

public class NoteMapManager extends StateMachine<NoteMapState> {
  private final RobotCommands actions;
  private final NoteTrackingManager noteTrackingManager;
  private final RobotManager robotManager;
  private final LocalizationSubsystem localization;
  private final SnapManager snaps;
  private final HeuristicPathFollowing pathfinder;

  private static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(5.0, 5.0, 2 * Math.PI, 4 * Math.PI);
  private static final double TARGET_NOTE_THRESHOLD_METERS = 1.5;
  private static final double INTAKE_PATHFIND_THRESHOLD_METERS = 2.0;
  private static final double DROPPED_NOTE_DISTANCE_METERS = 0.95;
  private static final double CLEANUP_SEARCH_THRESHOLD_METERS = 15.00;

  public NoteMapManager(
      RobotCommands actions,
      NoteTrackingManager noteTrackingManager,
      RobotManager robotManager,
      LocalizationSubsystem localization,
      SnapManager snaps) {
    super(SubsystemPriority.AUTOS, NoteMapState.STOPPED);
    this.actions = actions;
    this.noteTrackingManager = noteTrackingManager;
    this.robotManager = robotManager;
    this.localization = localization;
    this.snaps = snaps;
    this.pathfinder = new HeuristicPathFollowing(localization);
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
      if (RobotConfig.IS_HOME) {
        return NoteMapLocations.RED_DROPPING_DESTINATIONS_HOME;
      }
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

  private static Pose2d getSpeakerPose() {
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

  public Command dropNote() {
    return actions
        .dropCommand()
        .andThen(
            Commands.runOnce(
                () -> {
                  var translationFieldRelative =
                      new Translation2d(DROPPED_NOTE_DISTANCE_METERS, 0)
                          .rotateBy(localization.getPose().getRotation())
                          .plus(localization.getPose().getTranslation());

                  noteTrackingManager.addNoteToMap(translationFieldRelative);
                  AutoNoteDropped.addDroppedNote(translationFieldRelative);
                }));
  }

  public Command testCommand() {
    return Commands.runOnce(
        () -> {
          var now = Timer.getFPGATimestamp();
          DogLog.timestamp("AutoManager/TestCommandRun");
          noteTrackingManager.resetNoteMap(
              new ArrayList<>(
                  List.of(
                      new NoteMapElement(now + 20, AutoNoteStaged.noteIdToTranslation(4)),
                      new NoteMapElement(now + 20, AutoNoteStaged.noteIdToTranslation(5)))));
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

  // TODO: Remove this if we end up using triggers
  private Command noteMapCommand = Commands.none();

  public void setSteps(LinkedList<AutoNoteStep> newSteps) {
    steps = newSteps;
    setStateFromRequest(NoteMapState.WAITING_FOR_NOTES);
  }

  public void off() {
    currentStep = Optional.empty();
    steps = new LinkedList<>();
    maybeNotePose = Optional.empty();
    noteMapCommand.cancel();
    setStateFromRequest(NoteMapState.STOPPED);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    var translation = new Translation2d(12.85, 6.33);
    var translation1 = new Translation2d(12.8, 6.38);
DogLog.log("aaaaaa;lrkjqeroijqty", translation.getDistance(translation1));


    if (currentStep.isPresent()) {
      DogLog.log("AutoManager/CurrentStep/Action", currentStep.get().action());
    } else {
      DogLog.log("AutoManager/CurrentStep/Action", "NO_CURRENT_STEP");
    }
    pathfinder.temporaryLogFunction();

    switch (getState()) {
      case PATHFIND_TO_SCORE -> {
        if (robotManager.getState() == RobotState.IDLE_WITH_GP) {
          robotManager.waitSpeakerShotRequest();
        }
      }
      case INTAKING_PATHFINDING -> {
        if (maybeNotePose.isPresent()) {
          snaps.setAngle(maybeNotePose.get().getRotation().getDegrees());
        }
      }
      case PATHFIND_TO_DROP -> {
        if (robotManager.getState() == RobotState.IDLE_WITH_GP) {
          robotManager.waitingDropRequest();
        }
      }
      default -> {}
    }

    // todo: log target score pose, drop pose, etc.
    // log always, even when it's not relevant
    DogLog.log("AutoManager/ScoringPose", closestScoringLocation);
    DogLog.log("AutoManager/DroppingPose", droppingDestination);
  }

  @Override
  protected void collectInputs() {

    if (currentStep.isPresent()) {
      if (currentStep.get().action() == AutoNoteAction.CLEANUP) {
        var maybeNote =
            noteTrackingManager.getNoteNearPose(
                localization.getPose().getTranslation(), CLEANUP_SEARCH_THRESHOLD_METERS);
        if (maybeNote.isPresent()) {
          var noteDistanceAngle =
              VisionSubsystem.distanceAngleToTarget(
                  new Pose2d(maybeNote.get().noteTranslation(), new Rotation2d()),
                  localization.getPose());

          maybeNotePose =
              Optional.of(
                  new Pose2d(
                      maybeNote.get().noteTranslation(),
                      Rotation2d.fromDegrees(noteDistanceAngle.targetAngle() + 180)));

          if (maybeNotePose.isPresent()) {
            DogLog.log("AutoManager/MaybeNotePose", maybeNotePose.get());
          }
        } else {
          maybeNotePose = Optional.empty();
        }
      }

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
        snaps.setEnabled(false);
        DogLog.timestamp("AutoManager/IdleAction");
        if (steps.isEmpty()) {
          // Nothing new to do, keep idling
          DogLog.timestamp("AutoManager/IdleStepsEmpty");

        } else {
          DogLog.timestamp("AutoManager/IdleStepsPresent");
          currentStep = Optional.of(steps.poll());
          if (currentStep.isEmpty()) {
            DogLog.timestamp("AutoManager/IdleCurrentStepEmpty");
          } else {
            DogLog.timestamp("AutoManager/IdleCurrentStepPresent");
          }
        }
      }
      case DROP -> {
        noteMapCommand.cancel();

        snaps.setAngle(droppingDestination.getRotation().getDegrees());
        snaps.setEnabled(true);
        var translationFieldRelative =
            new Translation2d(DROPPED_NOTE_DISTANCE_METERS, 0)
                .rotateBy(localization.getPose().getRotation())
                .plus(localization.getPose().getTranslation());

        DogLog.log("Debug/droppednotepose", new Pose2d(translationFieldRelative, new Rotation2d()));
        noteTrackingManager.addNoteToMap(translationFieldRelative);
        AutoNoteDropped.addDroppedNote(translationFieldRelative);
        robotManager.dropRequest();
      }
      case SCORE -> {
        noteMapCommand.cancel();
        // If pathfinding ends sometimes there is a remaining chassis speeds
        // This sets that to zero, so the robot doesn't sorta drift around
        robotManager.swerve.setFieldRelativeSpeeds(new ChassisSpeeds(), true);
        robotManager.speakerShotRequest();
      }
      case INTAKING_PATHFINDING -> {
        noteMapCommand.cancel();
        robotManager.intakeRequest();
        snaps.setEnabled(true);
        if (currentStep.isEmpty()) {
          DogLog.timestamp("AutoManager/InPathActionCurrentStepEmpty");
          break;
        }

        if (maybeNotePose.isPresent()) {
          DogLog.timestamp("AutoManager/InPathActionNoteExists");
          noteMapCommand =
              robotManager
                  .swerve
                  .driveToPoseCommand(
                      () -> Optional.of(pathfinder.getPoseToFollow(maybeNotePose.get())),
                      localization::getPose,
                      false)
                  .withName("PathfindIntake");
          noteMapCommand.schedule();
        } else {
          DogLog.timestamp("AutoManager/InPathActionNoNoteExists");
        }
      }
      case INTAKING_PID -> {
        noteMapCommand.cancel();
        DogLog.log(
            "AutoManager/IntakePid/OldPathfindCommandCancelled", noteMapCommand.isFinished());
        DogLog.log(
            "AutoManager/IntakePid/OldPathfindCommandScheduled", noteMapCommand.isScheduled());
        snaps.setEnabled(false);
        if (currentStep.isEmpty()) {
          DogLog.timestamp("AutoManager/InPIDActionCurrentStepEmpty");
          break;
        }

        if (maybeNotePose.isPresent()) {
          DogLog.timestamp("AutoManager/InPIDActionNoteExists");
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
        snaps.setEnabled(false);

        droppingDestination = getClosestDroppingDestination();
        noteMapCommand =
            robotManager
                .swerve
                .driveToPoseCommand(
                    () -> Optional.of(pathfinder.getPoseToFollow(droppingDestination)),
                    localization::getPose,
                    false)
                .withName("PathfindDrop");
        noteMapCommand.schedule();
      }
      case PATHFIND_TO_SCORE -> {
        noteMapCommand.cancel();
        snaps.setEnabled(false);
        closestScoringLocation = getClosestScoringDestination();
        noteMapCommand =
            robotManager
                .swerve
                .driveToPoseCommand(
                    () -> Optional.of(pathfinder.getPoseToFollow(closestScoringLocation)),
                    localization::getPose,
                    false)
                .withName("PathfindScore");
        noteMapCommand.schedule();
      }
    }
  }

  private final Debouncer droppedNoteInRobotDebouncer = new Debouncer(0.5);

  // State transitions
  @Override
  protected NoteMapState getNextState(NoteMapState currentState) {
    return switch (currentState) {
      case STOPPED -> {
        yield currentState;
      }
      case WAITING_FOR_NOTES -> {
        if (currentStep.isPresent()
            && currentStep.get().action() == AutoNoteAction.CLEANUP
            && maybeNotePose.isPresent()) {
          DogLog.timestamp("AutoManager/IdleToCleanup");
          yield NoteMapState.INTAKING_PATHFINDING;
        } else if (currentStep.isPresent() && maybeNotePose.isPresent()) {
          DogLog.timestamp("AutoManager/IdleToIntakePathfinding");

          yield NoteMapState.INTAKING_PATHFINDING;
        }

        DogLog.timestamp("AutoManager/IdleToIdle");
        yield currentState;
      }
      case INTAKING_PATHFINDING -> {
        // If current step is empty
        if (currentStep.isEmpty()) {
          DogLog.timestamp("AutoManager/IntakingPathfindCurrentStepEmpty");

          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If note doesn't exist on map
        if (maybeNotePose.isEmpty()) {
          DogLog.timestamp("AutoManager/IntakingPathfindMapNoteGone");
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If we already have note go and score/drop
        if (robotManager.getState().hasNote) {
          DogLog.timestamp("AutoManager/IntakingPathfindRobotHasNote");
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
          DogLog.timestamp("AutoManager/CloseEnoughStopPathfinding");

          yield NoteMapState.INTAKING_PID;
        }

        // Pathfinding might finish even though we are way off from where we want to be
        // Which sucks but like, what're you gonna do
        // So just try PID at that point
        if (noteMapCommand.isFinished()) {
          DogLog.timestamp("AutoManager/IntakePathfindingFinished");
          yield NoteMapState.INTAKING_PID;
        }

        if (timeout(4)) {
          DogLog.timestamp("AutoManager/PathfindIntakeTimeout");
          yield NoteMapState.WAITING_FOR_NOTES;
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

        if (timeout(4)) {
          DogLog.timestamp("AutoManager/PIDIntakeTimeout");
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
        if (noteMapCommand.isFinished() || localization.atTranslation(droppingDestination.getTranslation(), 0.2)) {
          DogLog.timestamp("AutoManager/PathfindDropFinished");
          yield NoteMapState.DROP;
        }

        if (timeout(4)) {
          DogLog.timestamp("AutoManager/PathfindDropTimeout");
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
        if (noteMapCommand.isFinished() || localization.atTranslation(closestScoringLocation.getTranslation(), 0.2)) {
          DogLog.timestamp("AutoManager/PathfindScoreFinished");
          DogLog.log("AutoManager/PathfindToScore/Scheduled", noteMapCommand.isScheduled());
          DogLog.log("AutoManager/PathfindToScore/Finished", noteMapCommand.isFinished());
          yield NoteMapState.SCORE;
        }

        if (timeout(4)) {
          DogLog.timestamp("AutoManager/PathfindScoreTimeout");
          yield NoteMapState.SCORE;
        }
        yield currentState;
      }
      case DROP ->
          droppedNoteInRobotDebouncer.calculate(robotManager.getState().hasNote)
              ? currentState
              : NoteMapState.WAITING_FOR_NOTES;
      case SCORE -> {
        if (currentStep.isPresent() && currentStep.get().action().equals(AutoNoteAction.CLEANUP)) {
          if (robotManager.getState().hasNote) {
            yield currentState;
          }
          if (maybeNotePose.isPresent()) {
            yield NoteMapState.INTAKING_PATHFINDING;
          }
        }
        if (robotManager.getState().hasNote) {
          yield currentState;
        }

        yield NoteMapState.WAITING_FOR_NOTES;
      }
    };
  }
}

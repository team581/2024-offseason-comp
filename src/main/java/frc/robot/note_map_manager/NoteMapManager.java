// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager;

import com.pathplanner.lib.path.PathConstraints;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
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
  private static final double TIME_TO_WAIT_AFTER_DROPPING_NOTE = 0.5;
  private static final int MAX_ANGLE_TO_TARGET_BEFORE_DRIVING = 20;
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
  private static final double DROPPED_NOTE_DISTANCE_METERS = 0.8;
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

  /**
   * @return The angle the robot should be at to intake at the target pose
   */
  private double angleToIntake(Translation2d target) {
    return 180 + VisionSubsystem.angleToTarget(localization.getPose().getTranslation(), target);
  }

  private Pose2d getClosestScoringDestination() {
    Pose2d current = localization.getPose();

    var locations = NoteMapLocations.getScoringDestinations();
    Pose2d closest = locations.get(0);
    double currentDistance = Double.POSITIVE_INFINITY;

    for (Pose2d target : locations) {
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
    var locations = NoteMapLocations.getDroppingDestinations();

    Pose2d closest = locations.get(0);
    double currentDistance = Double.POSITIVE_INFINITY;

    for (Pose2d target : locations) {
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
        // This logic is unfortunately duplicated between this command (used in autos), and the
        // state machine stuff (used during note map)
        .andThen(Commands.waitSeconds(TIME_TO_WAIT_AFTER_DROPPING_NOTE))
        .finallyDo(
            () -> {
              DogLog.timestamp("AutoManager/DropNote/AddToMap");
              var translationFieldRelative =
                  new Translation2d(DROPPED_NOTE_DISTANCE_METERS, 0)
                      .rotateBy(localization.getPose().getRotation())
                      .plus(localization.getPose().getTranslation());

              // confirm the translation is correct
              DogLog.log("AutoManager/DropNote/NotePose", translationFieldRelative);

              noteTrackingManager.addNoteToMap(15, translationFieldRelative);
              AutoNoteDropped.addDroppedNote(translationFieldRelative);
            });
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

  private Optional<Translation2d> maybeNoteTranslation = Optional.empty();

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
    maybeNoteTranslation = Optional.empty();
    noteMapCommand.cancel();
    setStateFromRequest(NoteMapState.STOPPED);
  }

  @Override
  public void robotPeriodic() {
    if (DriverStation.isTeleop() && !RobotConfig.get().perfToggles().noteMapInTeleop()) {
      return;
    }

    super.robotPeriodic();

    if (currentStep.isPresent()) {
      DogLog.log("AutoManager/Steps/CurrentAction", currentStep.get().action());
    } else {
      DogLog.log("AutoManager/Steps/CurrentAction", "NO_CURRENT_STEP");
    }
    DogLog.log("AutoManager/Steps/Size", steps.size());
    pathfinder.temporaryLogFunction();

    switch (getState()) {
      case PATHFIND_TO_SCORE -> {
        if (robotManager.getState() == RobotState.IDLE_WITH_GP) {
          robotManager.waitSpeakerShotRequest();
        }
      }
      case INTAKING, INITIAL_AIM_TO_INTAKE -> {
        if (maybeNoteTranslation.isPresent()) {
          snaps.setAngle(angleToIntake(maybeNoteTranslation.get()));
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
    if (currentStep.isEmpty()) {
      return;
    }

    // Always reset the current note we are going after
    maybeNoteTranslation = Optional.empty();

    // Loop through the current step
    // This is kinda wasteful, since we don't always need to fully recalculate everything
    // It is easy though

    // Another downside is if we say score(4, 5), and 4 isn't there initially, but shows up later
    // It will forget about 5 and start going for 4 again
    // This is bad but probably won't happen in a match and we don't have time to make it more
    // robust
    for (var maybeSearchPoseSupplier : currentStep.get().notes()) {
      // Take each search pose supplier and check if we can get a search pose
      // Dropped notes might note have a search pose if we didn't drop them successfully
      var maybeSearchPose = maybeSearchPoseSupplier.get();
      if (maybeSearchPose.isEmpty()) {
        continue;
      }

      // Take the search location, and check if we have a note there
      var rawSearchLocation = maybeSearchPose.get();
      var maybeFoundNote =
          noteTrackingManager.getNoteNearPose(rawSearchLocation, TARGET_NOTE_THRESHOLD_METERS);

      // If that search location didn't have a note near it, try the next one
      if (maybeFoundNote.isEmpty()) {
        continue;
      }

      // If we found a note, set the current note to that note
      maybeNoteTranslation = Optional.of(maybeFoundNote.get().noteTranslation());

      DogLog.log("AutoManager/MaybeNoteTranslation", maybeNoteTranslation.get());
      break;
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

        // Check if the current step has any notes we can intake
        // If it does, stay on the step and try those
        // If not, try the next step
        if (currentStep.isPresent()
            && currentStep.get().notes().stream()
                .anyMatch(
                    maybeSearchPoseSupplier -> {
                      var maybeSearchPose = maybeSearchPoseSupplier.get();
                      if (maybeSearchPose.isEmpty()) {
                        return false;
                      }

                      var rawSearchLocation = maybeSearchPose.get();
                      return noteTrackingManager
                          .getNoteNearPose(rawSearchLocation, TARGET_NOTE_THRESHOLD_METERS)
                          .isPresent();
                    })) {
          // We have a note to intake, stay on the current step
        } else {
          // No notes to intake, try the next step
          currentStep = Optional.ofNullable(steps.poll());
        }

        // If there is no current step, stop the swerve
        if (currentStep.isEmpty()) {
          robotManager.swerve.setFieldRelativeSpeeds(new ChassisSpeeds(), true);
        }
        DogLog.timestamp("AutoManager/WaitingForNotesAction");
      }
      case DROP -> {
        noteMapCommand.cancel();

        snaps.setAngle(droppingDestination.getRotation().getDegrees());
        snaps.setEnabled(true);
        robotManager.swerve.setRobotRelativeSpeeds(new ChassisSpeeds(), true);
        var translationFieldRelative =
            new Translation2d(DROPPED_NOTE_DISTANCE_METERS, 0)
                .rotateBy(localization.getPose().getRotation())
                .plus(localization.getPose().getTranslation());

        DogLog.log("Debug/droppednotepose", new Pose2d(translationFieldRelative, new Rotation2d()));
        noteTrackingManager.addNoteToMap(15, translationFieldRelative);
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
      case INITIAL_AIM_TO_INTAKE -> {
        noteMapCommand.cancel();
        robotManager.intakeRequest();
        snaps.setEnabled(true);
        // Stop the swerve to avoid drifting slowly while we rotate
        // Snaps will override the omega here
        robotManager.swerve.setRobotRelativeSpeeds(new ChassisSpeeds(), true);
      }
      case INTAKING -> {
        noteMapCommand.cancel();
        robotManager.intakeRequest();
        snaps.setEnabled(true);
        if (currentStep.isEmpty()) {
          DogLog.timestamp("AutoManager/InPathActionCurrentStepEmpty");
          break;
        }

        if (maybeNoteTranslation.isPresent()) {
          DogLog.timestamp("AutoManager/InPathActionNoteExists");
          noteMapCommand =
              robotManager
                  .swerve
                  .driveToPoseCommand(
                      () ->
                          Optional.of(
                              pathfinder.getPoseToFollow(
                                  new Pose2d(
                                      maybeNoteTranslation.get(),
                                      Rotation2d.fromDegrees(
                                          angleToIntake(maybeNoteTranslation.get()))))),
                      localization::getPose,
                      false)
                  .withName("PathfindIntake");
          noteMapCommand.schedule();
        } else {
          DogLog.timestamp("AutoManager/InPathActionNoNoteExists");
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

  private final Debouncer droppedNoteInRobotDebouncer =
      new Debouncer(TIME_TO_WAIT_AFTER_DROPPING_NOTE);

  // State transitions
  @Override
  protected NoteMapState getNextState(NoteMapState currentState) {
    return switch (currentState) {
      case STOPPED -> {
        yield currentState;
      }
      case WAITING_FOR_NOTES -> {
        if (currentStep.isPresent() && maybeNoteTranslation.isPresent()) {
          if (!MathUtil.isNear(
              angleToIntake(maybeNoteTranslation.get()),
              localization.getPose().getRotation().getDegrees(),
              MAX_ANGLE_TO_TARGET_BEFORE_DRIVING)) {
            yield NoteMapState.INITIAL_AIM_TO_INTAKE;
          }

          yield NoteMapState.INTAKING;
        }

        DogLog.timestamp("AutoManager/IdleToIdle");
        yield currentState;
      }
      case INITIAL_AIM_TO_INTAKE -> {
        if (currentStep.isEmpty() || maybeNoteTranslation.isEmpty()) {
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

        if (timeout(1)
            || MathUtil.isNear(
                angleToIntake(maybeNoteTranslation.get()),
                localization.getPose().getRotation().getDegrees(),
                MAX_ANGLE_TO_TARGET_BEFORE_DRIVING)) {
          yield NoteMapState.INTAKING;
        }

        yield currentState;
      }
      case INTAKING -> {
        // If current step is empty
        if (currentStep.isEmpty()) {
          DogLog.timestamp("AutoManager/IntakingPathfindCurrentStepEmpty");

          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If note doesn't exist on map
        if (maybeNoteTranslation.isEmpty()) {
          DogLog.timestamp("AutoManager/IntakingPathfindMapNoteGone");
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If we already have note go and score/drop
        if (robotManager.getState().hasNote) {
          if (maybeNoteTranslation.isPresent()
              && localization.atTranslation(maybeNoteTranslation.get(), 0.5)) {
            noteTrackingManager.removeNote(maybeNoteTranslation.get(), 0.5);
          }
          DogLog.timestamp("AutoManager/IntakingPathfindRobotHasNote");
          if (currentStep.isPresent() && currentStep.get().action() == AutoNoteAction.DROP) {
            yield NoteMapState.PATHFIND_TO_DROP;
          }
          yield NoteMapState.PATHFIND_TO_SCORE;
        }

        if (noteMapCommand.isFinished()) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        if (timeout(5)) {
          DogLog.timestamp("AutoManager/PathfindIntakeTimeout");
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // Have a shorter timeout once we are about to get the note
        if (localization.atTranslation(maybeNoteTranslation.get(), 1) && timeout(1)) {
          DogLog.timestamp("AutoManager/PathfindIntakeFinalTimeout");
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
        if (noteMapCommand.isFinished()
            || localization.atTranslation(droppingDestination.getTranslation(), 0.2)) {
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
        if (noteMapCommand.isFinished()
            || localization.atTranslation(closestScoringLocation.getTranslation(), 0.2)) {
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
      case DROP -> {
        var debouncedHasNoteAfterDrop =
            droppedNoteInRobotDebouncer.calculate(robotManager.getState().hasNote);

        if (debouncedHasNoteAfterDrop) {
          // Still have note
          yield currentState;
        }

        // Finished this step since we dropped the note
        // Pop it off the head of the steps array
        steps.poll();

        yield NoteMapState.WAITING_FOR_NOTES;
      }
      case SCORE -> {
        if (currentStep.isPresent() && currentStep.get().action().equals(AutoNoteAction.CLEANUP)) {
          if (robotManager.getState().hasNote) {
            yield currentState;
          }
          if (maybeNoteTranslation.isPresent()) {
            yield NoteMapState.INTAKING;
          }
        }
        if (robotManager.getState().hasNote) {
          yield currentState;
        }

        // Finished scoring the note, remove this step
        steps.poll();
        yield NoteMapState.WAITING_FOR_NOTES;
      }
    };
  }
}

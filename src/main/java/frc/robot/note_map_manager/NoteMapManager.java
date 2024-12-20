// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  private static final int MAX_ANGLE_TO_TARGET_BEFORE_DRIVING = 20;
  private final RobotCommands actions;
  private final NoteTrackingManager noteTrackingManager;
  private final RobotManager robotManager;
  private final LocalizationSubsystem localization;
  private final SnapManager snaps;
  private final HeuristicPathFollowing pathfinder;

  private static final double TARGET_NOTE_THRESHOLD_METERS = 1.5;
  private static final double DROPPED_NOTE_DISTANCE_METERS = 0.88;
  private static final double DROPPED_NOTE_MOVING_DISTANCE_METERS = 0.55;
  private static final double ROBOT_AT_INTAKE_POSE_THESHOLD_METERS = 0.3;
  private static final double ROBOT_AT_DROP_POSE_THRESHOLD = 0.3;
  private static final double ROBOT_AT_SCORING_POSE_THRESHOLD = 0.3;

  private final Debouncer robotShouldHaveIntakedNoteDebouncer =
      new Debouncer(0.4, DebounceType.kBoth);
  private final Debouncer waitForFullyIntakedDebouncer = new Debouncer(0.2, DebounceType.kFalling);

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

  private Pose2d getClosestScoringLocation() {
    Pose2d current = localization.getPose();

    var locations = NoteMapLocations.getScoringDestinations();
    if (preferredScoringLocations.size() > 0) {
      locations = preferredScoringLocations;
    }
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

  private Pose2d getClosestDroppingLocation() {
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

  public Command dropNoteMovingBackward() {
    return actions
        .dropCommand()
        .finallyDo(
            (interrupted) -> {
              var translationFieldRelative =
                  new Translation2d(DROPPED_NOTE_MOVING_DISTANCE_METERS, 0)
                      .rotateBy(localization.getPose().getRotation())
                      .plus(localization.getPose().getTranslation());
              noteTrackingManager.addNoteToMap(15, translationFieldRelative);
              AutoNoteDropped.addDroppedNote(translationFieldRelative);
              DogLog.logFault("DropCommandInterrupted");
            });
  }

  public Command testCommand() {
    return Commands.runOnce(
        () -> {
          var now = Timer.getFPGATimestamp();
          noteTrackingManager.resetNoteMap(
              new ArrayList<>(
                  List.of(new NoteMapElement(now + 20, AutoNoteStaged.noteIdToTranslation(5)))));
          var steps = new LinkedList<AutoNoteStep>();
          steps.add(AutoNoteStep.score(5));
          setSteps(steps);
        });
  }

  private Optional<Translation2d> maybeNoteTranslation = Optional.empty();

  private Queue<AutoNoteStep> steps = new LinkedList<>();
  private Optional<AutoNoteStep> currentStep = Optional.empty();

  private List<Pose2d> preferredScoringLocations = List.of();
  private Pose2d scoringLocation = new Pose2d();
  private Pose2d droppingLocation = new Pose2d();
  private Command noteMapCommand = Commands.none();
  private int stepsOriginalSize = 0;
  private boolean debouncedHasNote = true;

  public void setSteps(LinkedList<AutoNoteStep> newSteps) {
    steps = newSteps;
    stepsOriginalSize = newSteps.size();
    DogLog.log("NoteMapManager/StepsOriginalSize", stepsOriginalSize);
    setStateFromRequest(NoteMapState.WAITING_FOR_NOTES);
  }

  public void setPreferredScoringLocations(List<Pose2d> scoringLocations) {
    preferredScoringLocations = scoringLocations;
  }

  public void clearPreferredScoringLocations() {
    preferredScoringLocations = List.of();
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
    DogLog.log("NoteMapManager/Steps/StepsLeft", steps.size());
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
    DogLog.log("NoteMapManager/Locations/ScoringLocation", scoringLocation);
    DogLog.log("NoteMapManager/Locations/DroppingLocation", droppingLocation);
  }

  @Override
  protected void collectInputs() {
    debouncedHasNote = hasNote();

    if (currentStep.isEmpty()) {
      return;
    }

    // Hacky way to start slowing down the shooter while intaking a note we know will be dropped
    robotManager.shooter.setEvilDropNoteHack(currentStep.get().action() == AutoNoteAction.DROP);

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
        DogLog.log("NoteMapManager/Status", "CurrentStepNoteEmpty");
        continue;
      }

      // Take the search location, and check if we have a note there
      var rawSearchLocation = maybeSearchPose.get();
      var maybeFoundNote =
          noteTrackingManager.getNoteNearPose(rawSearchLocation, TARGET_NOTE_THRESHOLD_METERS);

      // If that search location didn't have a note near it, try the next one
      if (maybeFoundNote.isEmpty()) {
        DogLog.log("NoteMapManager/Status", "FoundNoteEmpty");
        continue;
      }

      // If we found a note, set the current note to that note
      maybeNoteTranslation = Optional.of(maybeFoundNote.get().noteTranslation());

      DogLog.log("NoteMapManager/MaybeNoteTranslation", maybeNoteTranslation.get());
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
        DogLog.log("NoteMapManager/Status", "Stopped");
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
          doNextStep();
        }

        // Stop the swerve since we don't know what to do
        if (steps.size() < stepsOriginalSize) {
          robotManager.swerve.setFieldRelativeSpeeds(new ChassisSpeeds(), false);
        }
      }
      case DROP -> {
        noteMapCommand.cancel();

        snaps.setAngle(droppingLocation.getRotation().getDegrees());
        snaps.setEnabled(true);
        robotManager.swerve.setRobotRelativeSpeeds(new ChassisSpeeds(), true);

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
          break;
        }

        if (maybeNoteTranslation.isPresent()) {
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
          DogLog.log("NoteMapManager/Status", "IntakingActionNoNote");
        }
      }
      case PATHFIND_TO_DROP -> {
        noteMapCommand.cancel();
        droppingLocation = getClosestDroppingLocation();
        snaps.setAngle(droppingLocation.getRotation().getDegrees());
        snaps.setEnabled(true);

        noteMapCommand =
            robotManager
                .swerve
                .driveToPoseCommand(
                    () -> Optional.of(pathfinder.getPoseToFollow(droppingLocation)),
                    localization::getPose,
                    false)
                .withName("PathfindDrop");
        noteMapCommand.schedule();
      }
      case PATHFIND_TO_SCORE -> {
        noteMapCommand.cancel();
        snaps.setEnabled(false);
        scoringLocation = getClosestScoringLocation();

        noteMapCommand =
            robotManager
                .swerve
                .driveToPoseCommand(
                    () -> Optional.of(pathfinder.getPoseToFollow(scoringLocation)),
                    localization::getPose,
                    false)
                .withName("PathfindScore");
        noteMapCommand.schedule();
      }
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
        if (currentStep.isPresent()) {
          if (maybeNoteTranslation.isEmpty()) {
            // If there is truly not a single thing we can do for this step, exit the step
            doNextStep();
            yield currentState;
          }

          if (!MathUtil.isNear(
              angleToIntake(maybeNoteTranslation.get()),
              MathUtil.inputModulus(localization.getPose().getRotation().getDegrees(), 0, 360),
              MAX_ANGLE_TO_TARGET_BEFORE_DRIVING)) {
            yield NoteMapState.INITIAL_AIM_TO_INTAKE;
          }

          yield NoteMapState.INTAKING;
        }

        // This should only happen when every step is completed
        DogLog.log("NoteMapManager/Status", "AllStepsCompleted");
        yield currentState;
      }
      case INITIAL_AIM_TO_INTAKE -> {
        if (currentStep.isEmpty() || maybeNoteTranslation.isEmpty()) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }
        // If we already have note go and score/drop
        if (debouncedHasNote) {
          DogLog.log("NoteMapManager/Status", "InitialAimHasNote");
          if (currentStep.isPresent() && currentStep.get().action() == AutoNoteAction.DROP) {
            yield NoteMapState.PATHFIND_TO_DROP;
          }
          yield NoteMapState.PATHFIND_TO_SCORE;
        }

        if (timeout(1)
            || MathUtil.isNear(
                angleToIntake(maybeNoteTranslation.get()),
                MathUtil.inputModulus(localization.getPose().getRotation().getDegrees(), 0, 360),
                MAX_ANGLE_TO_TARGET_BEFORE_DRIVING)) {
          yield NoteMapState.INTAKING;
        }

        yield currentState;
      }
      case INTAKING -> {
        // If current step is empty
        if (currentStep.isEmpty()) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }
        // If we already have note go and score/drop
        if (debouncedHasNote) {
          DogLog.log("NoteMapManager/Status", "IntakingGotNoteRemoveNote");
          // Potentially this has a false positive where we have intaked some random other note
          // which is unrelated to the current note. So this would remove that unrelated note. But
          // that shouldn't ever really happen unless some timeouts trigger.
          noteTrackingManager.removeNote(
              localization.getPose().getTranslation(), ROBOT_AT_INTAKE_POSE_THESHOLD_METERS);

          DogLog.log("NoteMapManager/Status", "IntakingGotNote");
          if (currentStep.isPresent() && currentStep.get().action() == AutoNoteAction.DROP) {
            yield NoteMapState.PATHFIND_TO_DROP;
          }
          yield NoteMapState.PATHFIND_TO_SCORE;
        }

        // If note doesn't exist on map
        if (maybeNoteTranslation.isEmpty()) {
          DogLog.log("NoteMapManager/Status", "IntakingNoteGone");
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        if (noteMapCommand.isFinished()) {
          // We should have the note, but don't so we remove it from the map
          noteTrackingManager.removeNote(
              localization.getPose().getTranslation(), ROBOT_AT_INTAKE_POSE_THESHOLD_METERS);
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        if (timeout(5)) {
          DogLog.log("NoteMapManager/Status", "IntakingGeneralTimeout");
          // We should have the note, but don't so we remove it from the map
          noteTrackingManager.removeNote(
              localization.getPose().getTranslation(), ROBOT_AT_INTAKE_POSE_THESHOLD_METERS);
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // Short timeout once we are about to intake a note
        if (robotShouldHaveIntakedNoteDebouncer.calculate(
            localization.atTranslation(
                maybeNoteTranslation.get(), ROBOT_AT_INTAKE_POSE_THESHOLD_METERS))) {
          DogLog.log("NoteMapManager/Status", "IntakingFinalTimeout");
          // We should have the note, but don't so we remove it from the map
          noteTrackingManager.removeNote(
              localization.getPose().getTranslation(), ROBOT_AT_INTAKE_POSE_THESHOLD_METERS);
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        DogLog.log("NoteMapManager/Status", "TryingToIntake");
        yield currentState;
      }
      case PATHFIND_TO_DROP -> {
        // If robot doesn't have a note, give up (go to next step)
        if (!debouncedHasNote) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // if we finished pathfinding, drop note
        if (noteMapCommand.isFinished()
            || localization.atTranslation(
                droppingLocation.getTranslation(), ROBOT_AT_DROP_POSE_THRESHOLD)) {
          yield NoteMapState.DROP;
        }

        if (timeout(4)) {
          DogLog.log("NoteMapManager/Status", "PathfindToDropTimeout");
          yield NoteMapState.DROP;
        }

        yield currentState;
      }
      case PATHFIND_TO_SCORE -> {
        // If robot doesn't have a note, give up (go to next step)
        if (!debouncedHasNote) {
          yield NoteMapState.WAITING_FOR_NOTES;
        }

        // If we're already at location to score, score the note
        if (noteMapCommand.isFinished()
            || localization.atTranslation(
                scoringLocation.getTranslation(), ROBOT_AT_SCORING_POSE_THRESHOLD)) {
          yield NoteMapState.SCORE;
        }

        if (timeout(4)) {
          DogLog.log("NoteMapManager/Status", "PathfindToScoreTimeout");
          yield NoteMapState.SCORE;
        }
        yield currentState;
      }
      case DROP -> {
        // Use the state machine, since that debounces the sensor to avoid flinging the note after
        // drop
        if (robotManager.getState().hasNote) {
          // Still have note
          yield currentState;
        }

        // Finished this step since we dropped the note
        // Pop it off the head of the steps array
        doNextStep();
        // Add the dropped note to the map
        var translationFieldRelative =
            new Translation2d(DROPPED_NOTE_DISTANCE_METERS, 0)
                .rotateBy(localization.getPose().getRotation())
                .plus(localization.getPose().getTranslation());

        noteTrackingManager.addNoteToMap(15, translationFieldRelative);
        AutoNoteDropped.addDroppedNote(translationFieldRelative);

        if (timeout(3)) {
          DogLog.log("NoteMapManager/Status", "DropTimeout");

          yield NoteMapState.WAITING_FOR_NOTES;
        }
        yield NoteMapState.WAITING_FOR_NOTES;
      }
      case SCORE -> {
        if (currentStep.isPresent() && currentStep.get().action().equals(AutoNoteAction.CLEANUP)) {
          if (debouncedHasNote) {
            yield currentState;
          }
          if (maybeNoteTranslation.isPresent()) {
            yield NoteMapState.INTAKING;
          }
        }
        if (debouncedHasNote) {
          yield currentState;
        }

        if (timeout(3)) {
          DogLog.log("NoteMapManager/Status", "ScoreTimeout");
          yield NoteMapState.WAITING_FOR_NOTES;
        }
        // Finished scoring the note, remove this step
        doNextStep();
        yield NoteMapState.WAITING_FOR_NOTES;
      }
    };
  }

  private boolean hasNote() {
    return waitForFullyIntakedDebouncer.calculate(
        robotManager.noteManager.queuer.hasNote() || robotManager.noteManager.intake.hasNote());
  }

  /** Pull next step from the array and use that as the current step. */
  private void doNextStep() {
    currentStep = Optional.ofNullable(steps.poll());
  }
}

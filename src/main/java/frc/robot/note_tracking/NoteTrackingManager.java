// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_tracking;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.Stopwatch;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class NoteTrackingManager extends LifecycleSubsystem {
  private static final double CAMERA_IMAGE_HEIGHT = 960.0;
  private static final double CAMERA_IMAGE_WIDTH = 1280.0;
  // how much we keep a note in the map if it was added or updated from camera (seconds)
  private static final double NOTE_MAP_LIFETIME = 10.0;
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final RobotCommands actions;
  private final RobotManager robot;
  private final SnapManager snaps;
  private static final String LIMELIGHT_NAME = "limelight-note";
  private final InterpolatingDoubleTreeMap tyToDistance = new InterpolatingDoubleTreeMap();
  private ArrayList<NoteMapElement> noteMap = new ArrayList<>();

  private static final double FOV_VERTICAL = 48.823;
  private static final double FOV_HORIZONTAL = 62.074;
  private static final double HORIZONTAL_LEFT_VIEW = 27.491;
  private static final double VERTICAL_TOP_VIEW = 24.955;

  public NoteTrackingManager(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      RobotCommands actions,
      RobotManager robot) {
    super(SubsystemPriority.NOTE_TRACKING);

    this.localization = localization;
    this.swerve = swerve;
    this.actions = actions;
    this.robot = robot;
    this.snaps = robot.snaps;
    RobotConfig.get().vision().tyToNoteDistance().accept(tyToDistance);
  }

  public void resetNoteMap(ArrayList<NoteMapElement> startingValues) {
    noteMap = startingValues;
  }

  /**
   * Get the pose of the note closest to the provided location, within a threshold. Returns
   * optional.empty if no notes are tracked or notes exceed the threshold.
   */
  private Optional<NoteMapElement> getNearestNotePoseRelative(
      Pose2d searchLocation, double thresholdMeters) {

    var maybeElement =
        noteMap.stream()
            .filter(
                element -> {
                  return element
                          .notePose()
                          .getTranslation()
                          .getDistance(searchLocation.getTranslation())
                      < thresholdMeters;
                })
            .min(
                (a, b) ->
                    Double.compare(
                        a.notePose().getTranslation().getDistance(searchLocation.getTranslation()),
                        b.notePose()
                            .getTranslation()
                            .getDistance(searchLocation.getTranslation())));

    if (!maybeElement.isPresent()) {
      return Optional.empty();
    }

    return Optional.of(maybeElement.get());
  }

  private Optional<Pose2d> noteTxTyToPose(double tx, double ty) {

    double latency =
        (LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME)
                + LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME))
            / 1000.0;
    double timestamp = Timer.getFPGATimestamp() - latency;

    Pose2d robotPoseAtCapture = localization.getPose(timestamp);

    if (tx == 0) {
      return Optional.empty();
    }


    double forwardDistanceToNote = tyToDistance.get(ty);
    Rotation2d angleFromNote = Rotation2d.fromDegrees(tx);

    var c = forwardDistanceToNote / Math.cos(angleFromNote.getRadians());
    double sidewaysDistanceToNote = Math.sqrt(Math.pow(c, 2) - Math.pow(forwardDistanceToNote, 2));

    // Flips side of robot note is on based on if tx is positive or negative
    if (tx > 0) {
      sidewaysDistanceToNote *= -1.0;
    }

    var notePoseWithoutRotation =
        new Translation2d(-forwardDistanceToNote, -sidewaysDistanceToNote)
            .rotateBy(Rotation2d.fromDegrees(robotPoseAtCapture.getRotation().getDegrees()));

    var notePoseWithRobot =
        new Translation2d(
            robotPoseAtCapture.getX() + notePoseWithoutRotation.getX(),
            robotPoseAtCapture.getY() + notePoseWithoutRotation.getY());
    // Uses distance angle math to aim, inverses the angle for intake

    DistanceAngle noteDistanceAngle =
        VisionSubsystem.distanceAngleToTarget(
            new Pose2d(notePoseWithRobot, new Rotation2d()), robotPoseAtCapture);
    Rotation2d rotation = new Rotation2d(noteDistanceAngle.targetAngle().getRadians() + Math.PI);

    return Optional.of(new Pose2d(notePoseWithRobot, rotation));
  }

  private List<Pose2d> getRawNotePoses() {
    List<Pose2d> notePoses = new ArrayList<>();
    double[] corners =
        NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("tcornxy")
            .getDoubleArray(new double[8]);
    // Loop through 4 points

    // Delete 3 point note data

    if (corners.length >= 8 && corners[0] != 0.0 && corners.length % 8 == 0) {

      for (int i = 0; i < corners.length; i = i + 8) {
        var centerX = (corners[i] + corners[i + 2]) / 2;
        var centerY = (corners[i + 1] + corners[i + 5]) / 2;

        var angleX = (((centerX / CAMERA_IMAGE_WIDTH) * FOV_HORIZONTAL) - HORIZONTAL_LEFT_VIEW);
        var angleY = -1 * (((centerY / CAMERA_IMAGE_HEIGHT) * FOV_VERTICAL) - VERTICAL_TOP_VIEW);

        DogLog.log("NoteTracking/angley", angleY);
        DogLog.log("NoteTracking/anglex", angleX);

        var maybeNotePose = noteTxTyToPose(angleX, angleY);

        if (maybeNotePose.isPresent()) {
          notePoses.add(maybeNotePose.get());
        }
      }
    }

    return notePoses;
  }

  private static boolean isOutOfBounds(Pose2d notePose) {
    var fieldBorderThreshold = Units.inchesToMeters(4);
    boolean yOutOfBounds =
        notePose.getY() < (0.0 + fieldBorderThreshold)
            || notePose.getY() > (8.2 - fieldBorderThreshold);
    boolean xOutofBounds =
        notePose.getX() < (0.0 + fieldBorderThreshold)
            || notePose.getX() > (16.51 - fieldBorderThreshold);

    return yOutOfBounds || xOutofBounds;
  }

  private List<Pose2d> getFilteredNotePoses() {
    if (!safeToTrack()) {
      return List.of();
    }
    List<Pose2d> possibleNotes = getRawNotePoses();
    List<Pose2d> filteredNotes = new ArrayList<>();

    for (Pose2d possibleNote : possibleNotes) {
      if (!isOutOfBounds(possibleNote)) {
        filteredNotes.add(possibleNote);
      }
    }

    return filteredNotes;
  }

  private boolean safeToTrack() {
    var speeds = swerve.getRobotRelativeSpeeds();
    // TODO: finish refactor for chassis speeds
    return speeds.vxMetersPerSecond < 2
        && speeds.vyMetersPerSecond < 2
        && speeds.omegaRadiansPerSecond < Units.degreesToRadians(3.0);
  }

  private void removeNote(NoteMapElement note) {
    noteMap.remove(note);
  }

  public Command intakeNoteAtPose(Pose2d searchPose, double thresholdMeters) {
    return intakeNoteAtPose(() -> searchPose, thresholdMeters);
  }

  public Command intakeNoteAtPose(Supplier<Pose2d> searchPose, double thresholdMeters) {
    return actions
        .intakeCommand()
        .alongWith(
            swerve.driveToPoseCommand(
                () -> {
                  var nearestNote = getNearestNotePoseRelative(searchPose.get(), thresholdMeters);

                  if (nearestNote.isPresent()) {
                    snaps.setAngle(nearestNote.get().notePose().getRotation());
                    snaps.setEnabled(true);
                    DogLog.log("Debug/IntakingOriginal", Timer.getFPGATimestamp());

                    return Optional.of(nearestNote.get().notePose());

                  } else {
                    snaps.setEnabled(false);
                    DogLog.log("Debug/IntakingCancelled", true);
                    return Optional.empty();
                  }
                },
                this::getPose,
                false))
        .until(
            () ->
                robot.getState() == RobotState.IDLE_WITH_GP
                    || getNearestNotePoseRelative(searchPose.get(), 1.5).isEmpty())
        .andThen(
            Commands.runOnce(
                () -> {
                  var intakedNote = getNearestNotePoseRelative(searchPose.get(), 0.5);
                  if (intakedNote.isPresent()) {
                    removeNote(intakedNote.get());
                  }
                }))
        .withName("IntakeNearestNoteCommand");
  }

  public Command intakeNearestMapNote(double thresholdMeters) {
    return intakeNoteAtPose(this::getPose, thresholdMeters);
  }

  private Pose2d getPose() {
    return localization.getPose();
  }

  @Override
  public void robotPeriodic() {
    DogLog.log(
        "NoteTracking/NotesExpired",
        noteMap.removeIf(
            element -> {
              DogLog.log("NoteTracking/ExpiredNote", element.notePose());
              return element.expiresAt() < Timer.getFPGATimestamp();
            }));

    DogLog.log(
        "NoteTracking/NoteMap",
        noteMap.stream().map(NoteMapElement::notePose).toArray(Pose2d[]::new));

    Stopwatch.getInstance().start("Debug/NoteMapTime");
    noteMap = getNewMap();
    Stopwatch.getInstance().stop("Debug/NoteMapTime");

    // log closest note to bobot
    var maybeClosest = getNearestNotePoseRelative(getPose(), 99987.0);
    if (maybeClosest.isPresent()) {

      DogLog.log("NoteTracking/ClosestNote", maybeClosest.get().notePose());
    }
  }

  public boolean mapContainsNote() {
    return noteMap.size() > 0.0;
  }

  private ArrayList<NoteMapElement> getNewMap() {
    List<Pose2d> visionNotes = getFilteredNotePoses();

    if (visionNotes.size() > 20) {
      // Something evil happened, don't update state
      return noteMap;
    }

    ArrayList<NoteMapElement> result = new ArrayList<>();

    result.addAll(
        visionNotes.stream()
            .map(pose -> new NoteMapElement(Timer.getFPGATimestamp() + NOTE_MAP_LIFETIME, pose))
            .toList());

    for (var rememberedPose : noteMap) {
      Optional<Pose2d> visionNoteMatch =
          visionNotes.stream()
              .filter(
                  visionNote -> {
                    return visionNote
                            .getTranslation()
                            .getDistance(rememberedPose.notePose().getTranslation())
                        < 1;
                  })
              .min(
                  (a, b) ->
                      Double.compare(
                          a.getTranslation()
                              .getDistance(rememberedPose.notePose().getTranslation()),
                          b.getTranslation()
                              .getDistance(rememberedPose.notePose().getTranslation())));

      if (visionNoteMatch.isPresent()) {

        visionNotes.remove(visionNoteMatch.get());
      } else {

        result.add(rememberedPose);
      }
    }

    return result;
  }
}

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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_manager.AutoNoteDropped;
import frc.robot.auto_manager.BoundingBox;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
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
  private static final NetworkTableEntry LL_TCORNXY =
      NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tcornxy");
  private final InterpolatingDoubleTreeMap tyToDistance = new InterpolatingDoubleTreeMap();
  private ArrayList<NoteMapElement> noteMap = new ArrayList<>();
  private BoundingBox cameraFieldBox;

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

  private boolean noteInView(Pose2d notePose) {
    return cameraFieldBox.contains(notePose.getTranslation());
  }

  private void updateBox() {
    Pose2d robotPose = localization.getPose();
    var tLB = new Pose2d(1, 0.5, robotPose.getRotation());
    var tRB = new Pose2d(1, -0.5, robotPose.getRotation());
    var bLB = new Pose2d(0.3, 0.3, robotPose.getRotation());
    var bRB = new Pose2d(0.3, -0.3, robotPose.getRotation());

    var topLeft = new Translation2d(robotPose.getX() + tLB.getX(), robotPose.getY() + tLB.getY());
    var topRight = new Translation2d(robotPose.getX() + tRB.getX(), robotPose.getY() + tRB.getY());
    var bottomLeft =
        new Translation2d(robotPose.getX() + bLB.getX(), robotPose.getY() + bLB.getY());
    var bottomRight =
        new Translation2d(robotPose.getX() + bRB.getX(), robotPose.getY() + bRB.getY());

    cameraFieldBox = new BoundingBox(topLeft, topRight, bottomLeft, bottomRight);
  }

  public void resetNoteMap(ArrayList<NoteMapElement> startingValues) {
    AutoNoteDropped.clearDroppedNotes();
    noteMap = startingValues;
  }

  /**
   * Get the pose of the note closest to the provided location, within a threshold. Returns
   * optional.empty if no notes are tracked or notes exceed the threshold.
   */
  public Optional<NoteMapElement> getNearestNotePoseRelative(
      Translation2d searchLocation, double thresholdMeters) {

    var maybeElement =
        noteMap.stream()
            .filter(
                element -> {
                  return element.noteTranslation().getDistance(searchLocation) < thresholdMeters;
                })
            .min(
                (a, b) ->
                    Double.compare(
                        a.noteTranslation().getDistance(searchLocation),
                        b.noteTranslation().getDistance(searchLocation)));

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
            .rotateBy(robotPoseAtCapture.getRotation());

    var notePoseWithRobot =
        new Translation2d(
            robotPoseAtCapture.getX() + notePoseWithoutRotation.getX(),
            robotPoseAtCapture.getY() + notePoseWithoutRotation.getY());
    // Uses distance angle math to aim, inverses the angle for intake

    DistanceAngle noteDistanceAngle =
        VisionSubsystem.distanceAngleToTarget(
            new Pose2d(notePoseWithRobot, new Rotation2d()), robotPoseAtCapture);
    Rotation2d rotation =
        new Rotation2d(Units.degreesToRadians(noteDistanceAngle.targetAngle()) + Math.PI);

    return Optional.of(new Pose2d(notePoseWithRobot, rotation));
  }

  private List<Pose2d> getRawNotePoses() {
    List<Pose2d> notePoses = new ArrayList<>();
    double[] corners = LL_TCORNXY.getDoubleArray(new double[0]);
    // Loop through 4 points

    // Delete 3 point note data

    if (corners.length >= 8 && corners[0] != 0.0 && corners.length % 8 == 0) {

      for (int i = 0; i < corners.length; i = i + 8) {
        var centerX = (corners[i] + corners[i + 2]) / 2.0;
        var centerY = (corners[i + 1] + corners[i + 5]) / 2.0;

        double angleX = (((centerX / CAMERA_IMAGE_WIDTH) * FOV_HORIZONTAL) - HORIZONTAL_LEFT_VIEW);
        double angleY =
            -1.0 * (((centerY / CAMERA_IMAGE_HEIGHT) * FOV_VERTICAL) - VERTICAL_TOP_VIEW);

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
    return speeds.vxMetersPerSecond < 2.5
        && speeds.vyMetersPerSecond < 2.5
        && speeds.omegaRadiansPerSecond < Units.degreesToRadians(3.0);
  }

  private void removeNote(NoteMapElement note) {
    noteMap.remove(note);
  }

  public Command intakeNoteAtPose(Translation2d searchPose, double thresholdMeters) {
    return intakeNoteAtPose(() -> searchPose, thresholdMeters);
  }

  public Command intakeNoteAtPose(Supplier<Translation2d> searchPose, double thresholdMeters) {
    return actions
        .intakeCommand()
        .alongWith(
            swerve.driveToPoseCommand(
                () -> {
                  var nearestNote = getNearestNotePoseRelative(searchPose.get(), thresholdMeters);

                  if (nearestNote.isPresent()) {
                    DistanceAngle noteDistanceAngle =
                        VisionSubsystem.distanceAngleToTarget(
                            new Pose2d(nearestNote.get().noteTranslation(), new Rotation2d()),
                            localization.getPose());
                    Rotation2d rotation =
                        new Rotation2d(
                            Units.degreesToRadians(noteDistanceAngle.targetAngle()) + Math.PI);
                    var notePose = new Pose2d(nearestNote.get().noteTranslation(), rotation);

                    snaps.setAngle(rotation.getDegrees());
                    snaps.setEnabled(true);

                    return Optional.of(notePose);

                  } else {
                    snaps.setEnabled(false);
                    return Optional.empty();
                  }
                },
                localization::getPose,
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
        .finallyDo(
            () -> {
              snaps.setEnabled(false);
            })
        .withTimeout(4.0)
        .withName("IntakeNearestNoteCommand");
  }

  public Command intakeNearestMapNote(double thresholdMeters) {
    return intakeNoteAtPose(() -> localization.getPose().getTranslation(), thresholdMeters);
  }

  @Override
  public void robotPeriodic() {
    if (!RobotConfig.get().perfToggles().noteMapInTeleop() && DriverStation.isTeleop()) {
      return;
    }

    DogLog.log(
        "NoteTracking/NoteMap",
        noteMap.stream().map(NoteMapElement::noteTranslation).toArray(Pose2d[]::new));
    updateBox();

    updateMap();

    var maybeClosest = getNearestNotePoseRelative(localization.getPose().getTranslation(), 99987.0);
    if (maybeClosest.isPresent()) {

      DogLog.log("NoteTracking/ClosestNote", maybeClosest.get().noteTranslation());
    }
  }

  public boolean mapContainsNote() {
    return !noteMap.isEmpty();
  }

  public void addNoteToMap(Translation2d pose) {
    noteMap.add(new NoteMapElement(Timer.getFPGATimestamp() + NOTE_MAP_LIFETIME, pose));
  }

  private void updateMap() {
    List<Pose2d> visionNotes = getFilteredNotePoses();

    // 1. Remove expired notes since that's easy

    noteMap.removeIf(
        element -> {
          return element.expiresAt() < Timer.getFPGATimestamp();
        });

    // 2. Go through vision notes, and if it matches a note already in note map, replace it with the
    // vision note
    //    Otherwise, if we see a note with no match in the note map, add it to the array
    // 3. Don't do anything to the remembered notes otherwise. They stick around till expired

    for (var visionNote : visionNotes) {
      Optional<NoteMapElement> match =
          noteMap.stream()
              .filter(
                  rememberedNote -> {
                    return rememberedNote.noteTranslation().getDistance(visionNote.getTranslation())
                        < 1.0;
                  })
              .min(
                  (a, b) ->
                      Double.compare(
                          a.noteTranslation().getDistance(visionNote.getTranslation()),
                          b.noteTranslation().getDistance(visionNote.getTranslation())));

      if (match.isPresent()) {
        noteMap.remove(match.get());
      }

      noteMap.add(
          new NoteMapElement(
              Timer.getFPGATimestamp() + NOTE_MAP_LIFETIME, visionNote.getTranslation()));
    }
  }
}

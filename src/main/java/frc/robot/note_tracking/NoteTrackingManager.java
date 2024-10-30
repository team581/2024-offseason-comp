// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_tracking;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_map_manager.AutoNoteDropped;
import frc.robot.note_map_manager.BoundingBox;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class NoteTrackingManager extends LifecycleSubsystem {
  private static final double CAMERA_IMAGE_HEIGHT = 960.0;
  private static final double CAMERA_IMAGE_WIDTH = 1280.0;
  private static final double NOTE_LIMELIGHT_PITCH = 0.418879;
  private static final double NOTE_LIMELIGHT_HEIGHT = 0.4140708;
  private static final double NOTE_LIMELIGHT_FORWARD_DISTANCE_FROM_CENTER = 0.3683;

  // how much we keep a note in the map if it was added or updated from camera (seconds)
  private static final double NOTE_MAP_LIFETIME_SECONDS = 10.0;
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

  private static final double FOV_VERTICAL = 48.823;
  private static final double FOV_HORIZONTAL = 62.074;
  private static final double HORIZONTAL_LEFT_VIEW = 27.491;
  private static final double VERTICAL_TOP_VIEW = 24.955;

  private static final BoundingBox ROBOT_RELATIVE_FOV_BOUNDS =
      new BoundingBox(
          // top left
          new Translation2d(-2.350, -0.9),
          // top right
          new Translation2d(-2.350, 0.9),
          // bottom left
          new Translation2d(-1.2, -0.05),
          // bottom right
          new Translation2d(-1.2, 0.05));

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

  private boolean noteInView(Translation2d fieldRelativeNote) {
    var robotRelativeNote = getRobotRelativeNote(fieldRelativeNote);
    DogLog.log(
        "NoteTracking/Debug/BoxContainsNote",
        ROBOT_RELATIVE_FOV_BOUNDS.contains(robotRelativeNote));
    return ROBOT_RELATIVE_FOV_BOUNDS.contains(robotRelativeNote);
  }

  private Translation2d getRobotRelativeNote(Translation2d fieldRelativeNote) {
    var robotPose = localization.getPose();
    Rotation2d negativeRobotRotation = robotPose.getRotation().unaryMinus();
    var robotRelativeNoteTranslation =
        fieldRelativeNote.minus(robotPose.getTranslation()).rotateBy(negativeRobotRotation);
    return robotRelativeNoteTranslation;
  }

  public void resetNoteMap(ArrayList<NoteMapElement> startingValues) {
    AutoNoteDropped.clearDroppedNotes();
    noteMap = startingValues;
  }

  /**
   * Get the pose of the note closest to the provided location, within a threshold. Returns
   * optional.empty if no notes are tracked or notes exceed the threshold.
   */
  public Optional<NoteMapElement> getNoteNearPose(
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

    // Convert tx and ty to angles, tx on the limelight does not follow RHR
    // convention, so we need to negate by one

    double thetaX = -1 * Units.degreesToRadians(tx);
    double thetaY = Units.degreesToRadians(ty);
    double adjustedThetaY = NOTE_LIMELIGHT_PITCH - thetaY;
    double yOffset =
        (NOTE_LIMELIGHT_HEIGHT / Math.tan(adjustedThetaY))
            + Math.abs(NOTE_LIMELIGHT_FORWARD_DISTANCE_FROM_CENTER);

    double xOffset = yOffset * Math.tan(thetaX);

    // invert both offsets since note is technically behind robot
    var robotRelativeNoteTranslation = new Translation2d(-yOffset, -xOffset);
    var fieldRelativeNoteTranslation =
        robotRelativeNoteTranslation
            .rotateBy(robotPoseAtCapture.getRotation())
            .plus(robotPoseAtCapture.getTranslation());
    var fieldRelativeNotePose = new Pose2d(fieldRelativeNoteTranslation, new Rotation2d());

    return Optional.of(fieldRelativeNotePose);
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
    // This is important in ignoring hallucinated notes from reflections in field border (ex. RSL)
    var fieldBorderThreshold = Units.inchesToMeters(3);
    boolean yOutOfBounds =
        notePose.getY() < (0.0 + fieldBorderThreshold)
            || notePose.getY() > (8.2 - fieldBorderThreshold);
    boolean xOutofBounds =
        notePose.getX() < (0.0 + fieldBorderThreshold)
            || notePose.getX() > (16.51 - fieldBorderThreshold);

    return yOutOfBounds || xOutofBounds;
  }

  private List<Pose2d> getFilteredNotePoses() {

    List<Pose2d> possibleNotes = getRawNotePoses();
    List<Pose2d> filteredNotes = new ArrayList<>();

    for (Pose2d possibleNote : possibleNotes) {
      if (!isOutOfBounds(possibleNote)) {
        filteredNotes.add(possibleNote);
      }
    }

    if (!safeToTrack()) {
      return List.of();
    }

    return filteredNotes;
  }

  private boolean safeToTrack() {
    var speeds = swerve.getRobotRelativeSpeeds();

    return speeds.vxMetersPerSecond < 3
        && speeds.vyMetersPerSecond < 3
        && speeds.omegaRadiansPerSecond < Units.degreesToRadians(3.0);
  }

  public void removeNote(Translation2d searchArea, double threshold) {
    var intakedNote = getNoteNearPose(searchArea, 1.5);
    if (intakedNote.isPresent()) {
      noteMap.remove(intakedNote.get());
    }
  }

  @Override
  public void robotPeriodic() {

    if (DriverStation.isTeleop() && !RobotConfig.get().perfToggles().noteMapInTeleop()) {
      return;
    }

    updateMap();

    try {
      DogLog.log(
          "NoteTracking/NoteMap",
          noteMap.stream()
              .map(element -> new Pose2d(element.noteTranslation(), new Rotation2d()))
              .toArray(Pose2d[]::new));
    } catch (Exception error) {
      DogLog.logFault("NoteMapLoggingError");
      System.err.println(error);
    }

    var fieldRelativeBounds = getFieldRelativeBounds();
    DogLog.log("NoteTracking/CameraBounds", fieldRelativeBounds.toArray(Pose2d[]::new));
  }

  private List<Pose2d> getFieldRelativeBounds() {
    var robotRelativeToFieldRelativeTransform =
        new Transform2d(new Pose2d(), localization.getPose());
    return ROBOT_RELATIVE_FOV_BOUNDS.getPoints().stream()
        .map(point -> point.plus(robotRelativeToFieldRelativeTransform))
        .toList();
  }

  public boolean mapContainsNote() {
    return !noteMap.isEmpty();
  }

  public void addNoteToMap(double lifetime, Translation2d pose) {
    noteMap.add(new NoteMapElement(Timer.getFPGATimestamp() + lifetime, pose));
  }

  private void updateMap() {
    List<Pose2d> visionNotes = getFilteredNotePoses();

    noteMap.removeIf(
        element -> {
          return (element.expiresAt() < Timer.getFPGATimestamp());
        });

    if (RobotConfig.get().perfToggles().noteMapBoundingBox() && safeToTrack()) {

      var filteredNotesInBox =
          noteMap.stream()
              .filter(
                  element -> {
                    return (noteInView(element.noteTranslation()));
                  })
              .toList();

      for (NoteMapElement noteMapElement : filteredNotesInBox) {
        noteMap.remove(noteMapElement);
        if (noteMapElement.health() > 1) {
          noteMap.add(
              new NoteMapElement(
                  noteMapElement.expiresAt(),
                  noteMapElement.noteTranslation(),
                  noteMapElement.health() - 1));
        }
      }
    }

    double newNoteExpiry = Timer.getFPGATimestamp() + NOTE_MAP_LIFETIME_SECONDS;

    for (var visionNote : visionNotes) {
      Optional<NoteMapElement> match =
          noteMap.stream()
              .filter(
                  rememberedNote -> {
                    return rememberedNote.expiresAt() != newNoteExpiry
                        && (rememberedNote
                                .noteTranslation()
                                .getDistance(visionNote.getTranslation())
                            < 1.0);
                  })
              .min(
                  (a, b) ->
                      Double.compare(
                          a.noteTranslation().getDistance(visionNote.getTranslation()),
                          b.noteTranslation().getDistance(visionNote.getTranslation())));

      if (match.isPresent()) {
        noteMap.remove(match.get());
      }

      noteMap.add(new NoteMapElement(newNoteExpiry, visionNote.getTranslation()));
    }
  }
}

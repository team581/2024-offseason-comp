// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_tracking;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
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

public class NoteTrackingManager extends LifecycleSubsystem {
  private static final double LIMELIGHT_VERTICAL_FOV_DEGREES = 25.0;
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final RobotCommands actions;
  private final RobotManager robot;
  private final SnapManager snaps;
  private static final String LIMELIGHT_NAME = "limelight-note";
  private final InterpolatingDoubleTreeMap tyToDistance = new InterpolatingDoubleTreeMap();
  private Optional<Pose2d> lastNotePose = Optional.empty();
  private ArrayList<NoteMapElement> noteMap = new ArrayList<>();

  private double noteCloseEnoughMeters = 1;

  private double midlineXValue = 8.3;
  Timer noteTrackTimer = new Timer();

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

  public Optional<Pose2d> getNearestNotePoseRelative(
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
                        a.notePose()
                        .getTranslation()
                        .getDistance(searchLocation.getTranslation()),
                        b.notePose()
                            .getTranslation()
                            .getDistance(searchLocation.getTranslation())));

    if (!maybeElement.isPresent()) {
      return Optional.empty();
    }

    return Optional.of(maybeElement.get().notePose());
  }

  private Optional<Pose2d> getNotePose() {
    double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
    double tx =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tx").getDouble(0);

    double latency =
        (LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME)
                + LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME))
            / 1000.0;
    double timestamp = Timer.getFPGATimestamp() - latency;

    Pose2d robotPoseAtCapture = localization.getPose(timestamp);

    if (tx == 0) {
      return Optional.empty();
    }

    if (ty < -15.0) {
      return Optional.empty();
    }

    DogLog.log("NoteTracking/TY", ty);
    DogLog.log("NoteTracking/TX", tx);
    DogLog.log("NoteTracking/LatencyRobotPose", robotPoseAtCapture);

    double forwardDistanceToNote = tyToDistance.get(ty);
    Rotation2d angleFromNote = Rotation2d.fromDegrees(tx);
    DogLog.log("NoteTracking/ForwardDistance", forwardDistanceToNote);

    var c = forwardDistanceToNote / Math.cos(angleFromNote.getRadians());
    double sidewaysDistanceToNote = Math.sqrt(Math.pow(c, 2) - Math.pow(forwardDistanceToNote, 2));

    // Flips side of robot note is on based on if tx is positive or negative
    if (tx > 0) {
      sidewaysDistanceToNote *= -1.0;
    }

    DogLog.log("NoteTracking/SidewaysDistance", sidewaysDistanceToNote);
    var notePoseWithoutRotation =
        new Translation2d(-forwardDistanceToNote, -sidewaysDistanceToNote)
            .rotateBy(Rotation2d.fromDegrees(robotPoseAtCapture.getRotation().getDegrees()));

    var notePoseWithRobot =
        new Translation2d(
            robotPoseAtCapture.getX() + notePoseWithoutRotation.getX(),
            robotPoseAtCapture.getY() + notePoseWithoutRotation.getY());
    // Uses distance angle math to aim, inverses the angle for intake

    DistanceAngle noteDistanceAngle =
        VisionSubsystem.distanceToTargetPose(
            new Pose2d(notePoseWithRobot, new Rotation2d()), robotPoseAtCapture);
    Rotation2d rotation = new Rotation2d(noteDistanceAngle.targetAngle().getRadians() + Math.PI);

    if (noteDistanceAngle.distance() < 1) {
      if (lastNotePose.isPresent()) {
        rotation = lastNotePose.get().getRotation();
      } else {
        rotation = getPose().getRotation();
      }
    }
    return Optional.of(new Pose2d(notePoseWithRobot, rotation));
  }

  private List<Pose2d> getRawNotePoses() {}

  public Command intakeDetectedNote() {
    return Commands.runOnce(() -> lastNotePose = getNotePose())
        .andThen(actions.intakeCommand())
        .raceWith(
            swerve.driveToPoseCommand(
                () -> {
                  if (lastNotePose.isPresent()) {
                    snaps.setAngle(lastNotePose.get().getRotation());
                    snaps.setEnabled(true);
                  } else {
                    snaps.setEnabled(false);
                  }

                  return lastNotePose;
                },
                this::getPose))
        .finallyDo(
            () -> {
              noteTrackTimer.reset();
              robot.stopIntakingRequest();
            });
  }

  private Pose2d getPose() {
    return localization.getPose();
  }

  @Override
  public void robotPeriodic() {
    noteMap.removeIf(element -> element.expiresAt() < Timer.getFPGATimestamp());

    // TODO: Log lifetimes at some point

    DogLog.log(
        "NoteTracking/NoteMap",
        noteMap.stream().map(NoteMapElement::notePose).toArray(Pose2d[]::new));

    noteMap = getNewMap();
  }

  private ArrayList<NoteMapElement> getNewMap() {
    // Take tracked notes, add them to note map, or update the note map entries
    var possibleMap = new ArrayList<>(noteMap);
    var rawMap = getRawNotePoses();
    ArrayList<NoteMapElement> updatedMap = new ArrayList<>();

    // 0. Create a result array (a new note map)
    // 1. Make a copy of note map, call it "possible matches"
    // 2. Loop through camera notes, find nearest match (within 1m)
    // 3. If you have a match, remove it from possible matches!!!
    // 4. Then, add the match to the result note map
    // 5. If you don't have a match, add to result note map
    // 6. set note map to equal result one

    for (var currentCameraPose : rawMap) {
      updatedMap.add(new NoteMapElement(Timer.getFPGATimestamp() + 5, currentCameraPose));

      // Is this adding a new note to the map, or should we remove one that is close to this
var existingMapElement = possibleMap.stream()
.filter(close -> {return close.notePose().getTranslation().getDistance(currentCameraPose.getTranslation())<1;})
.min(
                (a, b) ->
                    Double.compare(
                        a.notePose()
                        .getTranslation()
                        .getDistance(currentCameraPose.getTranslation()),
                        b.notePose()
                            .getTranslation()
                            .getDistance(currentCameraPose.getTranslation())));

  if (existingMapElement.isPresent()) {
    possibleMap.remove(existingMapElement.get());
  } else {
    // This is a new note, there are no nearby ones to match it to
  }


      // for (int j = 0; j < possibleMap.size(); j++) {
      //   var currentPossiblePose = possibleMap.get(j).notePose();
      //   if (currentPossiblePose.getTranslation().getDistance(currentCameraPose.getTranslation())
      //       < 1) {
      //     possibleMap.remove(j);
      //   }
      // }
    }
    return updatedMap;
  }
}

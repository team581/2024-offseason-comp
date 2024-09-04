// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_manager.AutoManager;
import frc.robot.auto_manager.AutoNoteStaged;
import frc.robot.auto_manager.AutoNoteStep;
import frc.robot.fms.FmsSubsystem;
import frc.robot.note_tracking.NoteMapElement;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.ArrayList;
import java.util.List;

public class AutoCommands {
  private static final boolean USE_DYNAMIC_AUTOS = true;

  private static Command followPathForAlliance(PathPlannerPath redPath, PathPlannerPath bluePath) {
    return Commands.either(
        AutoBuilder.followPath(redPath),
        AutoBuilder.followPath(bluePath),
        FmsSubsystem::isRedAlliance);
  }

  private final RobotCommands actions;
  private final RobotManager robotManager;
  private final AutoManager autoManager;
  private final NoteTrackingManager noteTrackingManager;

  public AutoCommands(
      RobotCommands actions,
      RobotManager robotManager,
      AutoManager autoManager,
      NoteTrackingManager noteTrackingManager) {
    this.actions = actions;
    this.robotManager = robotManager;
    this.autoManager = autoManager;
    this.noteTrackingManager = noteTrackingManager;
  }

  public Command doNothingCommand() {
    return robotManager.localization.getZeroCommand();
  }

  public Command speakerSnapCommand() {
    return actions.waitForSpeakerShotCommand();
  }

  public Command subwooferShotWithTimeout() {
    return actions
        .subwooferShotCommand()
        .withTimeout(2)
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("SubwooferShotWithTimeout");
  }

  public Command presetLeftShot() {
    return actions
        .presetLeftShotCommand()
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("PresetLeftShot");
  }

  public Command presetMiddleShot() {
    return actions
        .presetMiddleShotCommand()
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("PresetMiddleShot");
  }

  public Command speakerShotWithTimeout() {
    return actions
        .speakerShotCommand()
        .withTimeout(2)
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("SpeakerShotWithTimeout");
  }

  private boolean hasNote() {
    if (!USE_DYNAMIC_AUTOS) {
      return true;
    }

    return robotManager.noteManager.intake.sensorHasNote()
        || robotManager.noteManager.queuer.sensorHasNote()
        || robotManager.getState().hasNote
        || robotManager.getState() == RobotState.FINISH_INTAKING;
  }

  public Command getMidlineNotesAmp456Command() {
    var red4ToCenterWingShot = PathPlannerPath.fromChoreoTrajectory("Red 4 to CWS");
    var red4To5 = PathPlannerPath.fromChoreoTrajectory("Red 4 to 5");
    var redCenterWingShotTo5 = PathPlannerPath.fromChoreoTrajectory("Red CWS to 5");
    var red5ToCenterWingShot = PathPlannerPath.fromChoreoTrajectory("Red 5 to CWS");
    var red5To6 = PathPlannerPath.fromChoreoTrajectory("Red 5 to 6");
    var redCenterWingShotTo6 = PathPlannerPath.fromChoreoTrajectory("Red CWS to 6");
    var red6ToCenterWingShot = PathPlannerPath.fromChoreoTrajectory("Red 6 to CWS");

    // var blue4ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 4 to CWS");
    //  var blue4To5 = PathPlannerPath.fromPathFile("Blue 4 to 5");
    //   var blueCenterWingShotTo5 = PathPlannerPath.fromPathFile("Blue CWS to 5");
    //  var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    //  var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");
    //   var blueCenterWingShotTo6 = PathPlannerPath.fromPathFile("Blue CWS to 6");
    //  var blue6ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 6 to CWS");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red4ToCenterWingShot, red4ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redCenterWingShotTo5, redCenterWingShotTo5)),
            followPathForAlliance(red4To5, red4To5),
            this::hasNote),
        Commands.either(
                followPathForAlliance(red5ToCenterWingShot, red5ToCenterWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redCenterWingShotTo6, redCenterWingShotTo6)),
                followPathForAlliance(red5To6, red5To6),
                this::hasNote)
            .andThen(
                followPathForAlliance(red6ToCenterWingShot, red6ToCenterWingShot)
                    .andThen(speakerShotWithTimeout())));
  }

  public Command getMidlineNotesAmp45Command() {
    var red4ToCenterWingShot = PathPlannerPath.fromPathFile("Red 4 to CWS");
    var red4To5 = PathPlannerPath.fromPathFile("Red 4 to 5");
    var redCenterWingShotTo5 = PathPlannerPath.fromPathFile("Red CWS to 5");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");

    var blue4ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 4 to CWS");
    var blue4To5 = PathPlannerPath.fromPathFile("Blue 4 to 5");
    var blueCenterWingShotTo5 = PathPlannerPath.fromPathFile("Blue CWS to 5");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red4ToCenterWingShot, blue4ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redCenterWingShotTo5, blueCenterWingShotTo5)),
            followPathForAlliance(red4To5, blue4To5),
            this::hasNote),
        Commands.either(
            followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                .andThen(speakerShotWithTimeout()),
            followPathForAlliance(red5To6, blue5To6),
            this::hasNote));
  }

  public Command getMidlineNotesAltAmpCommand() {
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");
    var red6To4 = PathPlannerPath.fromPathFile("Red 6 to 4");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var redCenterWingShotTo6 = PathPlannerPath.fromPathFile("Red CWS to 6");
    var red6ToCenterWingShot = PathPlannerPath.fromPathFile("Red 6 to CWS");

    var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");
    var blue6To4 = PathPlannerPath.fromPathFile("Blue 6 to 4");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blueCenterWingShotTo6 = PathPlannerPath.fromPathFile("Blue CWS to 6");
    var blue6ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 6 to CWS");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redCenterWingShotTo6, blueCenterWingShotTo6)),
            followPathForAlliance(red5To6, blue5To6),
            this::hasNote),
        Commands.either(
            followPathForAlliance(red6ToCenterWingShot, blue6ToCenterWingShot)
                .andThen(speakerShotWithTimeout()),
            followPathForAlliance(red6To4, blue6To4),
            this::hasNote));
  }

  public Command getMidlineNotesOP4Command() {
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var redRightWingShotTo6 = PathPlannerPath.fromPathFile("Red RWS to 6");
    var red6ToCenterWingShot = PathPlannerPath.fromPathFile("Red 6 to CWS");
    var redCenterWingShotTo4 = PathPlannerPath.fromPathFile("Red CWS to 4");
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");
    var red6To4 = PathPlannerPath.fromPathFile("Red 6 to 4");
    var red4ToCenterWingShot = PathPlannerPath.fromPathFile("Red 4 to CWS");

    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blueRightWingShotTo6 = PathPlannerPath.fromPathFile("Blue RWS to 6");
    var blue6ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 6 to CWS");
    var blueCenterWingShotTo4 = PathPlannerPath.fromPathFile("Blue CWS to 4");
    var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");
    var blue6To4 = PathPlannerPath.fromPathFile("Blue 6 to 4");
    var blue4ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 4 to CWS");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redRightWingShotTo6, blueRightWingShotTo6)),
            followPathForAlliance(red5To6, blue5To6),
            this::hasNote),
        Commands.either(
            followPathForAlliance(red6ToCenterWingShot, blue6ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redCenterWingShotTo4, blueCenterWingShotTo4)),
            followPathForAlliance(red6To4, blue6To4),
            this::hasNote),
        Commands.either(
            followPathForAlliance(red4ToCenterWingShot, blue4ToCenterWingShot)
                .andThen(speakerShotWithTimeout()),
            actions.stowCommand(),
            this::hasNote));
  }

  public Command getMidlineNotesSource876Command() {
    var red8ToLeftWingShot = PathPlannerPath.fromPathFile("Red 8 to LWS");
    var redLeftWingShotTo7 = PathPlannerPath.fromPathFile("Red LWS to 7");
    var red7ToLeftWingShot = PathPlannerPath.fromPathFile("Red 7 to LWS");
    var redLeftWingShotTo6 = PathPlannerPath.fromPathFile("Red LWS to 6");
    var red6ToLeftWingShot = PathPlannerPath.fromPathFile("Red 6 to LWS");
    var red8To7 = PathPlannerPath.fromPathFile("Red 8 to 7");
    var red7To6 = PathPlannerPath.fromPathFile("Red 7 to 6");

    var blue8ToLeftWingShot = PathPlannerPath.fromPathFile("Blue 8 to LWS");
    var blueLeftWingShotTo7 = PathPlannerPath.fromPathFile("Blue LWS to 7");
    var blue7ToLeftWingShot = PathPlannerPath.fromPathFile("Blue 7 to LWS");
    var blueLeftWingShotTo6 = PathPlannerPath.fromPathFile("Blue LWS to 6");
    var blue6ToLeftWingShot = PathPlannerPath.fromPathFile("Blue 6 to LWS");
    var blue8To7 = PathPlannerPath.fromPathFile("Blue 8 to 7");
    var blue7To6 = PathPlannerPath.fromPathFile("Blue 7 to 6");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red8ToLeftWingShot, blue8ToLeftWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redLeftWingShotTo7, blueLeftWingShotTo7)),
            followPathForAlliance(red8To7, blue8To7),
            this::hasNote),
        Commands.either(
                followPathForAlliance(red7ToLeftWingShot, blue7ToLeftWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redLeftWingShotTo6, blueLeftWingShotTo6)),
                followPathForAlliance(red7To6, blue7To6),
                this::hasNote)
            .andThen(
                followPathForAlliance(red6ToLeftWingShot, blue6ToLeftWingShot)
                    .andThen(speakerShotWithTimeout())));
  }

  public Command doManyAutoSteps(List<AutoNoteStep> steps) {
    return Commands.sequence(steps.stream().map(autoManager::doAutoStep).toArray(Command[]::new));
  }

  public Command noteMap456Command() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              var now = Timer.getFPGATimestamp();
              noteTrackingManager.resetNoteMap(
                  new ArrayList<>(
                      List.of(
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(4)),
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(5)),
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(6)))));
            }),
        doManyAutoSteps(
            List.of(
                AutoNoteStep.dropPreload(),
                AutoNoteStep.score(4, 5),
                AutoNoteStep.score(5, 6),
                AutoNoteStep.score(6))));
  }

  public Command notemap567Command() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              var now = Timer.getFPGATimestamp();
              noteTrackingManager.resetNoteMap(
                  new ArrayList<>(
                      List.of(
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(5)),
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(6)),
                          new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(7)))));
            }),
        doManyAutoSteps(
            List.of(
                AutoNoteStep.dropPreload(),
                AutoNoteStep.score(5, 6),
                AutoNoteStep.score(6, 7),
                AutoNoteStep.score(7))));
  }

  public Command testAuto() {
    return Commands.sequence(
        doNothingCommand()
            .alongWith(
                actions
                    .homeCommand()
                    .andThen(
                        AutoBuilder.followPath(Paths.testPath)
                            .andThen(actions.speakerShotCommand()))));
  }
}

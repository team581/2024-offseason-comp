// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.note_map_manager.AutoNoteStaged;
import frc.robot.note_map_manager.AutoNoteStep;
import frc.robot.note_map_manager.NoteMapManager;
import frc.robot.note_tracking.NoteMapElement;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.ArrayList;
import java.util.LinkedList;
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
  private final NoteMapManager noteMapManager;
  private final NoteTrackingManager noteTrackingManager;

  public AutoCommands(
      RobotCommands actions,
      RobotManager robotManager,
      NoteMapManager noteMapManager,
      NoteTrackingManager noteTrackingManager) {
    this.actions = actions;
    this.robotManager = robotManager;
    this.noteMapManager = noteMapManager;
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

  public Command redAmpOPNM() {
    return Commands.runOnce(
        () -> {
          var steps = new LinkedList<AutoNoteStep>();
          steps.add(AutoNoteStep.score(3));
          steps.add(AutoNoteStep.drop(4, 5));
          steps.add(AutoNoteStep.score(5, 6));
          steps.add(AutoNoteStep.score(10));
          noteMapManager.setSteps(steps);
        });
  }

  public Command redMidDropRaceNM() {
    return Commands.runOnce(
        () -> {
          var steps = new LinkedList<AutoNoteStep>();
          steps.add(AutoNoteStep.drop(6));
          steps.add(AutoNoteStep.score(7, 8));
          steps.add(AutoNoteStep.score(11));
          steps.add(AutoNoteStep.score(10));
          noteMapManager.setSteps(steps);
        });
  }

  public Command noteMapResetCommand() {
    return Commands.runOnce(
        () -> {
          var now = Timer.getFPGATimestamp();
          noteTrackingManager.resetNoteMap(
              new ArrayList<>(
                  List.of(
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(4)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(1)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(2)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(3)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(5)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(6)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(7)),
                      new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(8)))));
        });
  }

  public Command waitingDropRequestCommand() {
    return Commands.runOnce(
        () -> {
          robotManager.waitingDropRequest();
        });
  }
}

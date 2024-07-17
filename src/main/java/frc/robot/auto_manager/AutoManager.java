// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.note_tracking.NoteMapElement;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import java.util.ArrayList;
import java.util.List;

public class AutoManager {
  private final RobotCommands actions;
  private final NoteTrackingManager noteTrackingManager;
  private final RobotManager robotManager;

  public AutoManager(
      RobotCommands actions, NoteTrackingManager noteTrackingManager, RobotManager robotManager) {
    this.actions = actions;
    this.noteTrackingManager = noteTrackingManager;
    this.robotManager = robotManager;
  }

  public Command doManyAutoSteps(List<AutoNoteStep> steps) {
    return Commands.sequence(steps.stream().map(this::doAutoStep).toArray(Command[]::new));
  }

  private Command doAutoStep(AutoNoteStep step) {
    var command = noteTrackingManager.intakeNoteAtPose(step.noteSearchPose().get());

    // TODO: Pathfind to outtake/shoot position
    if (step.action() == AutoNoteAction.OUTTAKE) {
      command =
          command.andThen(
              actions.shooterOuttakeCommand().unless(() -> !robotManager.getState().hasNote));
    } else if (step.action() == AutoNoteAction.SCORE) {
      command =
          command.andThen(
              actions.speakerShotCommand().unless(() -> !robotManager.getState().hasNote));
    }

    return command;
  }

  public Command testCommand() {

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              var now = Timer.getFPGATimestamp();
              // 1. reset note map as if it were start of auto
              noteTrackingManager.resetNoteMap(
                  new ArrayList<>(
                      List.of(
                          //   new NoteMapElement(now + 5, new Pose2d(8.271, 7.458, new
                          // Rotation2d(0))),
                          new NoteMapElement(now + 5, AutoNoteStep.noteIdToPose(4)),
                          new NoteMapElement(now + 5, AutoNoteStep.noteIdToPose(5)),
                          new NoteMapElement(now + 5, AutoNoteStep.noteIdToPose(6)))));
            }),
        // 2. intakes notes 4 5 6 and scores
        doManyAutoSteps(
            List.of(
                new AutoNoteStep(4, AutoNoteAction.OUTTAKE),
                new AutoNoteStep(5, AutoNoteAction.SCORE),
                new AutoNoteStep(6, AutoNoteAction.SCORE))));
  }
}

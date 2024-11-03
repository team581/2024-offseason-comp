// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_map_manager.NoteMapManager;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;

public class Autos extends LifecycleSubsystem {

  private static Command wrapAutoEvent(String commandName, Command command) {
    return Commands.sequence(
            Commands.print("[COMMANDS] Starting auto event " + commandName),
            command.deadlineWith(
                Commands.waitSeconds(5)
                    .andThen(
                        Commands.print(
                            "[COMMANDS] Auto event "
                                + commandName
                                + " has been running for 5+ seconds!"))),
            Commands.print("[COMMANDS] Finished auto event " + commandName))
        .handleInterrupt(() -> System.out.println("[COMMANDS] Cancelled auto event " + commandName))
        .withName(commandName);
  }

  private static void registerCommand(String eventName, Command command) {
    NamedCommands.registerCommand(eventName, wrapAutoEvent("Auto_" + eventName, command));
  }

  private final SwerveSubsystem swerve;
  private final AutoChooser autoChooser;
  private final AutoCommands autoCommands;
  private final NoteMapManager noteMapManager;
  private final NoteTrackingManager noteTrackingManager;

  public Autos(
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      RobotCommands actions,
      RobotManager robotManager,
      NoteMapManager noteMapManager,
      NoteTrackingManager noteTrackingManager) {
    super(SubsystemPriority.AUTOS);
    this.swerve = swerve;
    this.noteMapManager = noteMapManager;
    this.noteTrackingManager = noteTrackingManager;

    autoCommands = new AutoCommands(actions, robotManager, noteMapManager, noteTrackingManager);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        localization::getPose,
        localization::resetPose,
        swerve::getRobotRelativeSpeeds,
        (robotRelativeSpeeds) -> {
          swerve.setRobotRelativeSpeeds(robotRelativeSpeeds, true);
        },
        new HolonomicPathFollowerConfig(
            new PIDConstants(4.0, 0.0, 0.0),
            new PIDConstants(2.5, 0.0, 0.0),
            4.4,
            0.387,
            new ReplanningConfig(true, true)),
        () -> false,
        swerve);

    registerCommand("preloadNote", actions.preloadNoteCommand());
    registerCommand("speakerShotNoTimeout", actions.speakerShotCommand());
    registerCommand("preparePresetRightShot", actions.preparePresetRightShotCommand());
    registerCommand("presetRightShot", actions.presetRightShotCommand());
    registerCommand("preparePresetLeftShot", actions.preparePresetLeftShotCommand());
    registerCommand("presetLeftShot", actions.presetLeftShotCommand());
    registerCommand("preparePresetMiddleShot", actions.preparePresetMiddleShotCommand());
    registerCommand("presetMiddleShot", actions.presetMiddleShotCommand());
    registerCommand("preparePreset3Shot", actions.preparePreset3ShotCommand());
    registerCommand("preset3Shot", actions.preset3ShotCommand());
    registerCommand("speakerShot", autoCommands.speakerShotWithTimeout());
    registerCommand("forceSpeakerShot", actions.forceSpeakerShotCommand());
    registerCommand("subwooferShot", autoCommands.subwooferShotWithTimeout());
    registerCommand("intakeFloor", actions.intakeCommand());
    registerCommand("outtakeShooter", actions.outtakeShooterCommand());
    registerCommand("homeClimber", actions.homeCommand());
    registerCommand("stow", actions.stowCommand());
    registerCommand("zeroGyro", autoCommands.doNothingCommand());
    registerCommand("waitingSpeakerShot", actions.waitForSpeakerShotCommand());
    registerCommand("redAmpOPNM", autoCommands.redAmpOPNM());
    registerCommand("blueAmpOPNM", autoCommands.blueAmpOPNM());
    registerCommand("redMidDropRaceNM", autoCommands.redMidDropRaceNM());
    registerCommand("blueMidDropRaceNM", autoCommands.blueMidDropRaceNM());
    registerCommand("redSourceSideRaceNM", autoCommands.redSourceSideRaceNM());
    registerCommand("blueSourceSideRaceNM", autoCommands.blueSourceSideRaceNM());

    registerCommand("dropNote", noteMapManager.dropNote());
    registerCommand("waitingDropRequest", autoCommands.waitingDropRequestCommand());
    registerCommand("resetNoteMap", autoCommands.noteMapResetCommand());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          DogLog.log(
              "Autos/Trajectory/ActivePath", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          DogLog.log("Autos/Trajectory/TargetPose", targetPose);
        });

    if (!RobotConfig.IS_DEVELOPMENT) {
      PPLibTelemetry.enableCompetitionMode();
    }

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    autoChooser = new AutoChooser(autoCommands);

    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command getAutoCommand() {
    return autoChooser.getAutoCommand();
  }

  private Optional<Rotation2d> getRotationTargetOverride() {
    // Some condition that should decide if we want to override rotation
    if (swerve.snapsEnabled()) {
      // Return an optional containing the rotation override (this should be a field relative
      // rotation)
      return Optional.of(Rotation2d.fromDegrees(swerve.snapAngle()));
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void disabledPeriodic() {
    // Constantly load the selected auto to avoid lag on auto init
    getAutoCommand();
  }
}

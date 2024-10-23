// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.localization.LocalizationSubsystem;
import java.util.Optional;
import java.util.function.Function;

/**
 * A wrapper around PathPlanner's pathfinding tech that gives us a nice API to get a path to a
 * target.
 */
public class PathfindUtil {
  private final LocalizationSubsystem localization;
  private final PathConstraints constraints;
  private GoalEndState goalEndState = new GoalEndState(0, new Rotation2d());
  private Optional<PathPlannerPath> cachedPath = Optional.empty();

  public PathfindUtil(LocalizationSubsystem localization, PathConstraints constraints) {
    this.localization = localization;
    this.constraints = constraints;
    Pathfinding.ensureInitialized();
  }

  public void startCalculatingPath(Pose2d goal, double endVelocity) {
    Pathfinding.setGoalPosition(goal.getTranslation());
    Pathfinding.setStartPosition(localization.getPose().getTranslation());
    cachedPath = Optional.empty();
    goalEndState = new GoalEndState(endVelocity, goal.getRotation());
  }

  public void startCalculatingPath(Pose2d goal) {
    startCalculatingPath(goal, 0);
  }

  /**
   * @return An optional that is empty if the path is still generating, or the generated path once
   *     ready.
   */
  public Optional<PathPlannerPath> getCalculatedPath() {
    if (Pathfinding.isNewPathAvailable()) {
      // Get the newly generated path and cache it
      cachedPath = Optional.of(Pathfinding.getCurrentPath(constraints, goalEndState));
    }

    // Return whatever cached path we may have
    return cachedPath;
  }

  /**
   * Pathfinding happens asynchronously. This is a command that runs as a callback once generation
   * is finished.
   */
  public Command withPathCommand(
      Pose2d goal, double endVelocity, Function<PathPlannerPath, Command> afterPathGeneration) {
    DogLog.log("AutoManager/Pathfinding/TargetPose", goal);
    return Commands.sequence(
        waitForGenerationCommand(goal, endVelocity),
        new ProxyCommand(() -> afterPathGeneration.apply(getCalculatedPath().get())));
  }

  public Command withPathCommand(
      Pose2d goal, Function<PathPlannerPath, Command> afterPathGeneration) {
    return withPathCommand(goal, 0, afterPathGeneration);
  }

  public Command waitForGenerationCommand(Pose2d goal, double endVelocity) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              startCalculatingPath(goal, endVelocity);
            }),
        Commands.waitUntil(() -> getCalculatedPath().isPresent()));
  }

  public Command waitForGenerationCommand(Pose2d goal) {
    return waitForGenerationCommand(goal, 0);
  }
}

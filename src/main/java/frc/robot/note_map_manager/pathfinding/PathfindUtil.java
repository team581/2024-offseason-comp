// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.localization.LocalizationSubsystem;

/**
 * A wrapper around PathPlanner's pathfinding tech that gives us a nice API to get a path to a
 * target.
 */
public class PathfindUtil {
  private final LocalizationSubsystem localization;
  private final PathConstraints constraints;

  public PathfindUtil(LocalizationSubsystem localization, PathConstraints constraints) {
    this.localization = localization;
    this.constraints = constraints;
    Pathfinding.ensureInitialized();
  }

  public PathPlannerPath getPathToTarget(Pose2d goal, double endVelocity) {
    Pathfinding.setGoalPosition(goal.getTranslation());
    Pathfinding.setStartPosition(localization.getPose().getTranslation());
    // TODO: Confirm that this path is instantly available after calling the function, and we don't
    // need to poll it repeatedly for the new path. If polling is needed, add new states to note map
    // to wait until the path has been generated.
    return Pathfinding.getCurrentPath(
        constraints, new GoalEndState(endVelocity, goal.getRotation()));
  }

  public PathPlannerPath getPathToTarget(Pose2d goal) {
    return getPathToTarget(goal, 0);
  }
}

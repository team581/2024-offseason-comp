// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Paths {
  private static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(5.20, 5.0, 2 * Math.PI, 4 * Math.PI);

  public static final PathPlannerPath testPath =
      new PathPlannerPath(
          PathPlannerPath.bezierFromPoses(
              new Pose2d(15.20, 5.57, Rotation2d.fromDegrees(0)),
              new Pose2d(13.66, 5.57, Rotation2d.fromDegrees(0))),
          DEFAULT_CONSTRAINTS,
          new GoalEndState(0.0, Rotation2d.fromDegrees(0)));
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import java.util.ArrayList;
import java.util.List;

public class HeuristicPathFollowing {
  // Truss is 12 in by 12 in, radius is around 8.5 in
  // Assumption is robot is 30 in by 30 in, radius is around 21.2 in
  // Each truss has radius of 29.7 in or 0.75438 m
  private static final double TRUSS_RADIUS = 0.75438 + 0.1; //  (truss + robot) + fudge factor

  private static final List<CollisionPoint> BLUE_COLLISION_POINTS =
      List.of(
          new CollisionPoint(new Translation2d(3.3274, 4.106), TRUSS_RADIUS),
          new CollisionPoint(new Translation2d(5.65658, 5.447374), TRUSS_RADIUS),
          new CollisionPoint(new Translation2d(5.65658, 2.764626), TRUSS_RADIUS));

  private static final List<CollisionPoint> RED_COLLISION_POINTS =
      List.of(
          new CollisionPoint(new Translation2d(13.2842, 4.106), TRUSS_RADIUS),
          new CollisionPoint(new Translation2d(10.95502, 5.447374), TRUSS_RADIUS),
          new CollisionPoint(new Translation2d(10.95502, 2.764626), TRUSS_RADIUS));

  private static final List<Translation2d> BLUE_INTERMEDIARY_POINTS =
      List.of(new Translation2d(13.0404, 7.001), new Translation2d(13.0404, 1.0));

  private static final List<Translation2d> RED_INTERMEDIARY_POINTS =
      List.of(new Translation2d(14.2596, 7.001), new Translation2d(14.2596, 1.0));

  private static List<CollisionPoint> getCollisionPoints() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_COLLISION_POINTS;
    } else {
      return BLUE_COLLISION_POINTS;
    }
  }

  private static List<Translation2d> getIntermediaryPoints() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_INTERMEDIARY_POINTS;
    } else {
      return BLUE_INTERMEDIARY_POINTS;
    }
  }

  private static boolean doesLineCollideWithCircle(
      Translation2d start, Translation2d end, CollisionPoint circle) {
    // Return True if there is a collision, False if not
    double lineVectorX = end.getX() - start.getX();
    double lineVectorY = end.getY() - start.getY();
    double circleVectorX = circle.point().getX() - start.getX();
    double circleVectorY = circle.point().getY() - start.getY();

    double dotProduct = lineVectorX * circleVectorX + lineVectorY * circleVectorY;

    double lineVectorMagnitude = Math.hypot(lineVectorX, lineVectorY);
    double circleVectorMagnitude = Math.hypot(circleVectorX, circleVectorY);

    // Angle = acos(docProduct / (lineVectorMagnitude * circleVectorMagnitude))
    // Closest Point on Line to Center of Circle = circleVectorMagnitude * sin(angle)
    // sin(acos(x)) = sqrt(1 - x^2) - gets rid of trig operations
    double x = dotProduct / (lineVectorMagnitude * circleVectorMagnitude);
    double closestPointOnLineToCircleDistance =
        circleVectorMagnitude * Math.sqrt(1 - Math.pow(x, 2));

    return closestPointOnLineToCircleDistance < circle.radius();
  }

  private static boolean doesCollisionExist(Translation2d start, Translation2d end) {
    // Return True if there is a collision between the two points and any Collision Points, False if
    // not
    for (CollisionPoint point : getCollisionPoints()) {
      if (doesLineCollideWithCircle(start, end, point)) {
        return true;
      }
    }
    return false;
  }

  private final LocalizationSubsystem localization;

  public HeuristicPathFollowing(LocalizationSubsystem localization) {
    this.localization = localization;
  }

  public Pose2d getPoseToFollow(Pose2d end) {
    var robot = localization.getPose();

    if (!doesCollisionExist(robot.getTranslation(), end.getTranslation())) {
      return end;
    }

    ArrayList<Translation2d> validIntermediaryPoints = new ArrayList<>();
    for (Translation2d intermediaryPoint : getIntermediaryPoints()) {
      if (doesCollisionExist(robot.getTranslation(), intermediaryPoint)) {
        continue;
      }
      if (doesCollisionExist(end.getTranslation(), intermediaryPoint)) {
        continue;
      }
      validIntermediaryPoints.add(intermediaryPoint);
    }

    if (validIntermediaryPoints.isEmpty()) {
      // This is the evil outcome, should probably crash code and delete notemap from
      // Rio SD card if this happens.
      return end;
    }

    Pose2d closestPoint = end;
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (Translation2d intermediaryPoint : validIntermediaryPoints) {
      double distanceToRobot = robot.getTranslation().getDistance(intermediaryPoint);
      double distanceToEnd = end.getTranslation().getDistance(intermediaryPoint);
      if (distanceToRobot + distanceToEnd < smallestDistance) {
        smallestDistance = distanceToRobot + distanceToEnd;
        closestPoint = new Pose2d(intermediaryPoint, end.getRotation());
      }
    }
    return closestPoint;
  }
}

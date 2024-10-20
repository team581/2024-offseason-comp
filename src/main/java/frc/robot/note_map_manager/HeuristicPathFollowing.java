// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_map_manager;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.fms.FmsSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

// Truss is 12 in by 12 in, radius is around 8.5 in
// Assumption is robot is 30 in by 30 in, radius is around 21.2 in
// Each truss has radius of 29.7 in or 0.75438 m

private class CollisionPoint {
  public static Translation2d point;
  public static double size;
  public CollisionPoint(Translation2d point, double radius) {
    this.point = point;
    this.radius = radius; // in meters
  }
}

public class HeuristicPathFollowing {

  private static double TRUSS_RADIUS = 0.75438 + 0.1 + ; //  (truss + robot) + fudge factor

  public static final List<CollisionPoint> BLUE_COLLISION_POINTS = List.of(
    CollisionPoint(Translation2d(3.3274, 4.106), TRUSS_RADIUS),
    CollisionPoint(Translation2d(5.65658, 5.447374), TRUSS_RADIUS),
    CollisionPoint(Translation2d(5.65658, 2.764626), TRUSS_RADIUS)
  );

  public static final List<CollisionPoint> RED_COLLISION_POINTS = List.of(
    CollisionPoint(Translation2d(13.2842, 4.106), TRUSS_RADIUS),
    CollisionPoint(Translation2d(10.95502, 5.447374), TRUSS_RADIUS),
    CollisionPoint(Translation2d(10.95502, 2.764626), TRUSS_RADIUS)
  );

  public HeuristicPathFollowing(){}

  public List<CollisionPoint> getCollisionPoints(){
    if (FmsSubsystem.isRedAlliance()) {
      return RED_COLLISION_POINTS;
    } else {
      return BLUE_COLLISION_POINTS;
    }
  }

  public static boolean doesLineCollideWithCircle(Translation2d start, Translation2d end, CollisionPoint circle){
    // Return True if there is a collision, False if not
    double lineVectorX = end.getX() - start.getX();
    double lineVectorY = end.getY() - start.getY();
    double circleVectorX = circle.point.getX() - start.getX();
    double circleVectorY = circle.point.getY() - start.getY();

    double dotProduct = lineVectorX * circleVectorX + lineVectorY * circleVectorY;

    double lineVectorMagnitude = Math.sqrt(Math.pow(lineVectorX, 2) + Math.pow(lineVectorY, 2));
    double circleVectorMagnitude = Math.sqrt(Math.pow(circleVectorX, 2) + Math.pow(circleVectorY, 2));

    // Angle = acos(docProduct / (lineVectorMagnitude * circleVectorMagnitude))
    // Closest Point on Line to Center of Circle = circleVectorMagnitude * sin(angle)
    // sin(acos(x)) = sqrt(1 - x^2) - gets rid of trig operations
    double x = dotProduct / (lineVectorMagnitude * circleVectorMagnitude)
    double closestPointOnLineToCircleDistance = circleVectorMagnitude * Math.sqrt(1 - Math.pow(x, 2));

    return closestPointOnLineToCircleDistance < circle.radius;
  }

  public boolean doesCollisionExists(Translation2d start, Translation2d end){
    // Return True if there is a collision between the two points and any Collision Points, False if not
    List<CollisionPoint> CollisionPoint = getCollisionPoints();
    for (CollisionPoint point : CollisionPoint) {
      if (doesLineCollideWithCircle(start, end, point)){
        return true;
      }
    }
    return false;
  }


}

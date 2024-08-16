// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class BoundingBox {

  private Pose2d[] points;

  public BoundingBox(Pose2d topLeft, Pose2d topRight, Pose2d bottomLeft, Pose2d bottomRight) {
    points = new Pose2d[] {bottomLeft, topLeft, topRight, bottomRight};
  }

  public boolean containsPose(Pose2d pose) {
    double angleSum = 0;
    for (int i = 0; i < points.length; i++) {
      Pose2d corner1 = points[i];
      Pose2d corner2 = points[(i + 1) % points.length];
      angleSum += calculateAngle(pose, corner1, corner2);
    }

    return Math.abs(angleSum - (2 * Math.PI)) < 0.0001;
  }

  private double calculateAngle(Pose2d point, Pose2d corner1, Pose2d corner2) {
    double a = corner1.getTranslation().getDistance(corner2.getTranslation());
    double b = corner1.getTranslation().getDistance(point.getTranslation());
    double c = corner2.getTranslation().getDistance(point.getTranslation());
    double cosA = (b * b + c * c - a * a) / (2 * b * c);
    return Math.acos(cosA);
  }
}

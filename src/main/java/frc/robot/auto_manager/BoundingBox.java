// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.apache.commons.geometry.euclidean.twod.Bounds2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.path.LinePath;
import org.apache.commons.numbers.core.Precision;
import org.apache.commons.numbers.core.Precision.DoubleEquivalence;

public class BoundingBox {
  private static final DoubleEquivalence PRECISION = Precision.doubleEquivalenceOfEpsilon(0.01);

  private static Vector2D translationToVector(Translation2d translation) {
    return Vector2D.of(translation.getX(), translation.getY());
  }

  private static Translation2d vectorToTranslation(Vector2D vector) {
    return new Translation2d(vector.getX(), vector.getY());
  }

  private final Bounds2D bounds;

  private final List<Pose2d> TEMPORARY;

  public BoundingBox(
      Translation2d topLeft,
      Translation2d topRight,
      Translation2d bottomLeft,
      Translation2d bottomRight) {
    bounds =
        LinePath.builder(PRECISION)
            .appendVertices(
                translationToVector(topLeft),
                translationToVector(topRight),
                translationToVector(bottomRight),
                translationToVector(bottomLeft))
            .build()
            .getBounds();
    TEMPORARY =
        List.of(
            new Pose2d(topLeft, new Rotation2d()),
            new Pose2d(topRight, new Rotation2d()),
            new Pose2d(bottomLeft, new Rotation2d()),
            new Pose2d(bottomRight, new Rotation2d()));
  }

  public boolean contains(Translation2d translation) {
    return bounds.contains(translationToVector(translation));
  }

  public List<Pose2d> getPoints() {
    return TEMPORARY;
    // return bounds.toRegion(PRECISION).getVertices().stream()
    //     .map(vertex -> new Pose2d(vectorToTranslation(vertex), new Rotation2d(0)))
    //     .toList();
  }
}

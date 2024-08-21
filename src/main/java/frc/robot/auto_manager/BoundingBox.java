// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_manager;

import edu.wpi.first.math.geometry.Translation2d;
import org.apache.commons.geometry.euclidean.twod.Bounds2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.path.LinePath;
import org.apache.commons.numbers.core.Precision;

public class BoundingBox {
  private static Vector2D translationToVector(Translation2d translation) {
    return Vector2D.of(translation.getX(), translation.getY());
  }

  private final Bounds2D bounds;

  public BoundingBox(
      Translation2d topLeft,
      Translation2d topRight,
      Translation2d bottomLeft,
      Translation2d bottomRight) {
    bounds =
        LinePath.builder(Precision.doubleEquivalenceOfEpsilon(0.01))
            .appendVertices(
                translationToVector(topLeft),
                translationToVector(topRight),
                translationToVector(bottomRight),
                translationToVector(bottomLeft))
            .build()
            .getBounds();
  }

  public boolean contains(Translation2d translation) {
    return bounds.contains(translationToVector(translation));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.interpolation.InterpolationUtil;
import frc.robot.vision.interpolation.VisionInterpolationData;
import java.util.List;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

class InterpolationUtilTest {
  /** Util for dealing with floating point precision loss. */
  private static Translation2d simplify(Translation2d translation) {
    // Round to 1cm precision
    return new Translation2d(
        Math.round(translation.getX() * 100.0) / 100.0,
        Math.round(translation.getY() * 100.0) / 100.0);
  }

  @Test
  void noopWhenNoDataPoints() {
    var visionPose = new Translation2d(1, 2);

    var result = InterpolationUtil.interpolateTranslation(List.of(), visionPose);

    Assertions.assertEquals(visionPose, result);
  }

  @Test
  void interpolatesWithTwoDataPoints() {
    var dataPoints =
        List.of(
            new VisionInterpolationData(new Translation2d(1, 1), new Translation2d(2, 2), "1"),
            new VisionInterpolationData(new Translation2d(3, 3), new Translation2d(4, 4), "2"));

    var result = InterpolationUtil.interpolateTranslation(dataPoints, new Translation2d(3, 3));

    Assertions.assertEquals(new Translation2d(2.0, 2.0), simplify(result));
  }
}

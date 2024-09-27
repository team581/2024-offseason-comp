// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public enum InterpolatedVisionDataset {
  HOME(
      List.of(
          new VisionInterpolationData(
              new Translation2d(15.2245, 5.522), new Translation2d(15.194, 5.634), "SUBWOOFER"),
          new VisionInterpolationData(
              new Translation2d(13.0745, 5.522),
              new Translation2d(13.125, 5.722),
              "PODIUM_SPEAKER_INTERSECTION"),
          new VisionInterpolationData(
              new Translation2d(11.059, 6.842),
              new Translation2d(11.16, 6.845),
              "WING_LINE_MIDDLE"),
          new VisionInterpolationData(
              new Translation2d(13.799, 4.202),
              new Translation2d(13.905, 4.361),
              "FRONT_PODIUM_MIDDLE")),
      List.of()),
  BELLARMINE(
      List.of(
          new VisionInterpolationData(
              new Translation2d(15.2245, 5.522), new Translation2d(15.125, 5.581), "SUBWOOFER"),
          new VisionInterpolationData(
              new Translation2d(13.0745, 5.522),
              new Translation2d(13.103, 5.560),
              "PODIUM_SPEAKER_INTERSECTION"),
          new VisionInterpolationData(
              new Translation2d(11.059, 6.842),
              new Translation2d(11.18, 6.932),
              "WING_LINE_MIDDLE"),
          new VisionInterpolationData(
              new Translation2d(13.799, 4.202),
              new Translation2d(13.67, 4.106),
              "FRONT_PODIUM_MIDDLE")),
      List.of()),
  CHEZY_CHAMPS(
      List.of(
          new VisionInterpolationData(
              new Translation2d(15.2245, 5.522), new Translation2d(999.0, 999.0), "RED_SUBWOOFER"),
          new VisionInterpolationData(
              new Translation2d(13.0745, 5.522),
              new Translation2d(999.0, 999.0),
              "RED_PODIUM_SPEAKER_INTERSECTION"),
          new VisionInterpolationData(
              new Translation2d(11.059, 6.842),
              new Translation2d(999.0, 999.0),
              "RED_WING_LINE_MIDDLE"),
          new VisionInterpolationData(
              new Translation2d(13.799, 4.202),
              new Translation2d(999.0, 999.0),
              "RED_FRONT_PODIUM_MIDDLE")),
      List.of(
          new VisionInterpolationData(
              new Translation2d(1.315, 5.522), new Translation2d(999.0, 999.0), "BLUE_SUBWOOFER"),
          new VisionInterpolationData(
              new Translation2d(3.465, 5.522),
              new Translation2d(999.0, 999.0),
              "BLUE_PODIUM_SPEAKER_INTERSECTION"),
          new VisionInterpolationData(
              new Translation2d(5.481, 6.842),
              new Translation2d(999.0, 999.0),
              "BLUE_WING_LINE_MIDDLE"),
          new VisionInterpolationData(
              new Translation2d(2.741, 4.202),
              new Translation2d(999.0, 999.0),
              "BLUE_FRONT_PODIUM_MIDDLE")));

  public final List<VisionInterpolationData> redSet;
  public final List<VisionInterpolationData> blueSet;

  InterpolatedVisionDataset(List<VisionInterpolationData> red, List<VisionInterpolationData> blue) {
    this.redSet = red;
    this.blueSet = blue;
  }
}

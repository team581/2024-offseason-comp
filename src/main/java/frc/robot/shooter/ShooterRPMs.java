// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

public class ShooterRPMs {
  public static final double FULLY_STOPPED = 0;
  public static final double IDLE = 1000;
  public static final double OUTTAKE = 2000;
  public static final double DROPPING = 400;
  public static final double SUBWOOFER = 3000;
  public static final double PODIUM = 4400;
  public static final double SHOOTER_AMP = 600;
  public static final double PRESET_RIGHT = 3000;
  public static final double PRESET_LEFT = 3000;
  public static final double PRESET_MIDDLE = 3000;
  public static final double PRESET_3 = 4400;

  public static final double AMP_TOLERANCE = 100;
  public static final double TOLERANCE = 125;
  public static final double FEEDING_TOLERANCE = 250;
  public static final double SPIN_RATIO = 4.0 / 5.0;

  private ShooterRPMs() {}
}

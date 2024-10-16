// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class ShooterSubsystem extends LifecycleSubsystem {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private boolean usingNoteSpin = true;
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
  private double speakerDistance = 0;
  private double floorSpotDistance = 0;
  private double goalRPM = 0;

  private double usedTolerance = ShooterRPMs.TOLERANCE;
  private final InterpolatingDoubleTreeMap speakerDistanceToRPM = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap floorSpotDistanceToRPM =
      new InterpolatingDoubleTreeMap();

  private ShooterMode goalMode = ShooterMode.IDLE;

  public ShooterSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SHOOTER);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    RobotConfig.get().shooter().speakerShotRpms().accept(speakerDistanceToRPM);
    RobotConfig.get().shooter().floorShotRpms().accept(floorSpotDistanceToRPM);

    leftMotor.getConfigurator().apply(RobotConfig.get().shooter().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().shooter().rightMotorConfig());
  }

  @Override
  public void robotPeriodic() {
    usingNoteSpin = true;
    switch (goalMode) {
      case SPEAKER_SHOT:
        goalRPM = speakerDistanceToRPM.get(speakerDistance);
        break;
      case SUBWOOFER_SHOT:
        goalRPM = ShooterRPMs.SUBWOOFER;
        break;
      case PODIUM_SHOT:
        goalRPM = ShooterRPMs.PODIUM;
        break;
      case FLOOR_SHOT:
        goalRPM = floorSpotDistanceToRPM.get(floorSpotDistance);
        break;
      case DROPPING:
        goalRPM = ShooterRPMs.DROPPING;
        usingNoteSpin = false;
        break;
      case OUTTAKE:
        goalRPM = ShooterRPMs.OUTTAKE;
        break;
      case SHOOTER_AMP:
        goalRPM = ShooterRPMs.SHOOTER_AMP;
        usingNoteSpin = false;
        break;
      case IDLE:
        goalRPM = ShooterRPMs.IDLE;
        usingNoteSpin = false;
        break;
      case FULLY_STOPPED:
        goalRPM = ShooterRPMs.FULLY_STOPPED;
        usingNoteSpin = false;
        break;
      default:
        break;
    }

    DogLog.log("Shooter/Mode", goalMode);
    DogLog.log("Shooter/GoalRPM", goalRPM);
    DogLog.log(
        "Shooter/GoalRPMForRightMotor", goalRPM * (usingNoteSpin ? ShooterRPMs.SPIN_RATIO : 1.0));
    DogLog.log("Shooter/LeftMotor/RPM", getRPM(leftMotor));
    DogLog.log("Shooter/RightMotor/RPM", getRPM(rightMotor));
    DogLog.log("Shooter/AtGoal", atGoal(goalMode));

    if (goalMode == ShooterMode.FULLY_STOPPED) {
      leftMotor.disable();
      rightMotor.disable();
    } else {
      leftMotor.setControl(velocityRequest.withVelocity((goalRPM) / 60));
      rightMotor.setControl(
          velocityRequest.withVelocity(
              (goalRPM * (usingNoteSpin ? ShooterRPMs.SPIN_RATIO : 1.0)) / 60));
    }
  }

  public boolean atGoal(ShooterMode mode) {
    if (mode != goalMode) {
      return false;
    }

    if (goalMode == ShooterMode.IDLE || goalMode == ShooterMode.FULLY_STOPPED) {
      return true;
    }

    var usedTolerance =
        goalMode == ShooterMode.FLOOR_SHOT ? ShooterRPMs.FEEDING_TOLERANCE : ShooterRPMs.TOLERANCE;

    if (Math.abs((goalRPM * (usingNoteSpin ? ShooterRPMs.SPIN_RATIO : 1.0)) - getRPM(rightMotor))
            < usedTolerance
        && Math.abs(goalRPM - getRPM(leftMotor)) < usedTolerance) {
      return true;
    }

    return false;
  }

  private double getRPM(TalonFX motor) {
    return motor.getVelocity().getValueAsDouble() * 60.0;
  }

  public void setGoalMode(ShooterMode newMode) {
    goalMode = newMode;
  }

  public ShooterMode getGoalMode() {
    return goalMode;
  }

  public void setSpeakerDistance(double distance) {
    speakerDistance = distance;
  }

  public void setFloorSpotDistance(double distance) {
    floorSpotDistance = distance;
  }
}

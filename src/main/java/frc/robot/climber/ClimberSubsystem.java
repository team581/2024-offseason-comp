// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends LifecycleSubsystem {
  private static final ClimberConfig CONFIG = RobotConfig.get().climber();
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private StrictFollower followRequest;
  private LinearFilter currentFilter = LinearFilter.movingAverage(CONFIG.currentTaps());
  private double goalDistance = 0.0;
  private PositionVoltage positionRequest = new PositionVoltage(goalDistance);

  private ClimberMode goalMode = ClimberMode.STOWED;
  private HomingState homingState = HomingState.NOT_HOMED;

  public ClimberSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.CLIMBER);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftMotor.getConfigurator().apply(CONFIG.leftMotorConfig());
    rightMotor.getConfigurator().apply(CONFIG.rightMotorConfig());

    followRequest = new StrictFollower(leftMotor.getDeviceID());
  }

  @Override
  public void enabledInit() {
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void robotPeriodic() {
    double rawCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    setGoalDistance(goalMode.position);

    switch (homingState) {
      case NOT_HOMED:
        leftMotor.disable();
        rightMotor.disable();
        break;
      case MID_MATCH_HOMING:
        leftMotor.setVoltage(CONFIG.homingVoltage());
        rightMotor.setControl(followRequest);
        if (filteredCurrent > CONFIG.homingCurrentThreshold()) {
          leftMotor.setPosition(
              inchesToRotations(RobotConfig.get().climber().minDistance()).getRotations());
          rightMotor.setPosition(
              inchesToRotations(RobotConfig.get().climber().minDistance()).getRotations());
          homingState = HomingState.HOMED;
        }
        break;
      case HOMED:
        leftMotor.setControl(
            positionRequest.withPosition(inchesToRotations(clamp(goalDistance)).getRotations()));
        rightMotor.setControl(followRequest);

        break;
      case PRE_MATCH_HOMING:
        throw new IllegalStateException("Climber can't do pre match homing");
    }

    Logger.recordOutput("Climber/GoalMode", goalMode);
    Logger.recordOutput("Climber/GoalDistance", goalDistance);
    Logger.recordOutput("Climber/HomingState", homingState);
    Logger.recordOutput("Climber/Left/Distance", getDistance(leftMotor));
    Logger.recordOutput(
        "Climber/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Climber/Left/Rotations", leftMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Climber/Left/Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Climber/Left/VelocityDistance",
        rotationsToInches(Rotation2d.fromRotations(leftMotor.getVelocity().getValueAsDouble())));
    Logger.recordOutput("Climber/Right/Distance", getDistance(rightMotor));
    Logger.recordOutput(
        "Climber/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Climber/Right/Rotations", rightMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Climber/Right/Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Climber/Right/VelocityDistance",
        rotationsToInches(Rotation2d.fromRotations(rightMotor.getVelocity().getValueAsDouble())));
  }

  public boolean atGoal(ClimberMode goal) {
    if (goalMode != goal) {
      return false;
    }
    if (goalMode == ClimberMode.STOWED) {
      return true;
    }

    double leftDistance = getDistance(leftMotor);
    double rightDistance = getDistance(rightMotor);
    if (Math.abs(leftDistance - goalDistance) < CONFIG.distanceTolerance()
        && Math.abs(rightDistance - goalDistance) < CONFIG.distanceTolerance()) {
      return true;
    }
    return false;
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public void resetHoming() {
    homingState = HomingState.NOT_HOMED;
  }

  public void startHoming() {
    homingState = HomingState.MID_MATCH_HOMING;
  }

  public double getDistance(TalonFX motor) {
    return rotationsToInches(Rotation2d.fromRotations(motor.getPosition().getValueAsDouble()));
  }

  private void setGoalDistance(double distance) {
    goalDistance = clamp(distance);
  }

  public void setGoalMode(ClimberMode mode) {
    goalMode = mode;
  }

  private static double clamp(double distance) {
    return MathUtil.clamp(distance, CONFIG.minDistance(), CONFIG.maxDistance());
  }

  // Tune the radius in inches later
  private static double rotationsToInches(Rotation2d angle) {
    return angle.getRotations() * (CONFIG.rotationsToDistance());
  }

  private static Rotation2d inchesToRotations(double inches) {
    return Rotation2d.fromRotations(inches / (CONFIG.rotationsToDistance()));
  }
}

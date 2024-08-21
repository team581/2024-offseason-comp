// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;

  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final PositionVoltage positionRequest = new PositionVoltage(ElevatorPositions.STOWED);
  private final Timer timer = new Timer();
  private boolean pulsing = false;

  // Homing
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = 0.0;
  private HomingState homingState = HomingState.PRE_MATCH_HOMING;

  private double goalHeight = ElevatorPositions.STOWED;

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR);

    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());

    this.motor = motor;
    timer.start();
  }

  @Override
  public void disabledPeriodic() {
    double currentHeight = getHeight();

    if (currentHeight < lowestSeenHeight) {
      lowestSeenHeight = currentHeight;
    }
  }

  @Override
  public void robotPeriodic() {
    if (goalHeight != ElevatorPositions.TRAP_SHOT) {
      setPulsing(false);
    }

    switch (homingState) {
      case NOT_HOMED:
        motor.setControl(coastNeutralRequest);
        break;
      case PRE_MATCH_HOMING:
        motor.setControl(brakeNeutralRequest);

        if (DriverStation.isDisabled()) {
          // Wait until enable to do homing code
        } else {

          if (!preMatchHomingOccured) {
            double homingEndPosition = RobotConfig.get().elevator().homingEndPosition();
            double homedPosition = homingEndPosition + (getHeight() - lowestSeenHeight);
            motor.setPosition(Units.degreesToRotations(inchesToRotations(homedPosition)));

            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
        break;
      case HOMED:
        {
          int slot = goalHeight == RobotConfig.get().elevator().minHeight() ? 1 : 0;
          double height = clampHeight(goalHeight);

          if (pulsing && timer.hasElapsed(RobotConfig.get().elevator().pulseDuration())) {
            if (timer.hasElapsed(RobotConfig.get().elevator().pulseDuration() * 2.0)) {
              height = clampHeight(height - 4);
              timer.reset();
            } else {
              // Do nothing, height stays the same
            }
          }

          if (DriverStation.isDisabled()) {
            motor.setControl(brakeNeutralRequest);
          } else {
            motor.setControl(
                positionRequest
                    .withSlot(slot)
                    .withPosition(Units.degreesToRotations(inchesToRotations(height))));
          }

          break;
        }
      case MID_MATCH_HOMING:
        throw new IllegalStateException("Elevator can't do mid match homing");
    }

    DogLog.log("Elevator/Height", getHeight());
    DogLog.log("Elevator/GoalHeight", goalHeight);
  }

  public void setPulsing(boolean shouldPulse) {
    pulsing = shouldPulse;
  }

  public void setGoalHeight(double newHeight) {
    goalHeight = clampHeight(newHeight);
  }

  public double getHeight() {
    return rotationsToInches(getMechanismRotations());
  }

  private double getMechanismRotations() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public boolean atPosition(double distance) {
    return Math.abs(getHeight() - distance) < RobotConfig.get().elevator().tolerance();
  }

  private static double rotationsToInches(double rotations) {
    return Units.degreesToRotations(rotations)
        * (RobotConfig.get().elevator().rotationsToDistance());
  }

  private static double inchesToRotations(double inches) {
    return Units.rotationsToDegrees(inches / (RobotConfig.get().elevator().rotationsToDistance()));
  }

  private static double clampHeight(double height) {
    return MathUtil.clamp(
        height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }
}

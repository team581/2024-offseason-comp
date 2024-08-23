// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class WristSubsystem extends LifecycleSubsystem {

  private final TalonFX motor;
  private final PositionVoltage positionRequest =
      new PositionVoltage(Units.degreesToRotations(WristPositions.STOWED)).withEnableFOC(true);

  private final CoastOut coastNeutralRequest = new CoastOut();

  private HomingState homingState = HomingState.PRE_MATCH_HOMING;

  private static final InterpolatingDoubleTreeMap speakerDistanceToAngle =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap floorSpotDistanceToAngle =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToAngleTolerance =
      new InterpolatingDoubleTreeMap();

  private double lowestSeenAngle = Double.MAX_VALUE;

  private boolean preMatchHomingOccured = false;

  private double goalAngle = 0;

  public WristSubsystem(TalonFX motor) {
    super(SubsystemPriority.WRIST);
    this.motor = motor;

    motor.getConfigurator().apply(RobotConfig.get().wrist().motorConfig());

    RobotConfig.get().wrist().speakerShotAngles().accept(speakerDistanceToAngle);
    RobotConfig.get().wrist().distanceToAngleTolerance().accept(distanceToAngleTolerance);
    RobotConfig.get().wrist().floorShotAngles().accept(floorSpotDistanceToAngle);
  }

  @Override
  public void disabledPeriodic() {
    double currentAngle = getAngle();

    if (currentAngle < lowestSeenAngle) {
      lowestSeenAngle = currentAngle;
    }
  }

  @Override
  public void robotPeriodic() {
    switch (homingState) {
      case NOT_HOMED:
        motor.setControl(coastNeutralRequest);
        break;
      case PRE_MATCH_HOMING:
        motor.disable();

        if (DriverStation.isEnabled() && !preMatchHomingOccured) {
          double homedAngle = getHomeAngleFromLowestSeen();
          motor.setPosition(Units.degreesToRotations(homedAngle));

          preMatchHomingOccured = true;
          homingState = HomingState.HOMED;
        }

        break;
      case MID_MATCH_HOMING:
        throw new IllegalStateException("Wrist can't do mid match homing");
      case HOMED:
        int slot = goalAngle == RobotConfig.get().wrist().minAngle() ? 1 : 0;

        motor.setControl(
            positionRequest
                .withSlot(slot)
                .withPosition(Units.degreesToRotations(clampAngle(goalAngle))));

        break;
    }

    DogLog.log("Wrist/Position", Units.rotationsToDegrees(motor.getPosition().getValue()));
    DogLog.log("Wrist/HomingState", homingState);
    DogLog.log("Wrist/GoalAngle", goalAngle);
  }

  private double getHomeAngleFromLowestSeen() {
    return (RobotConfig.get().wrist().homingEndPosition() + (getAngle() - lowestSeenAngle));
  }

  public void setAngle(double angle) {
    angle = clampAngle(angle);

    goalAngle = angle;
  }

  private static double clampAngle(double angle) {
    if (angle < RobotConfig.get().wrist().minAngle()) {
      angle = RobotConfig.get().wrist().minAngle();

    } else if (angle > RobotConfig.get().wrist().maxAngle()) {
      angle = RobotConfig.get().wrist().maxAngle();
    }
    return angle;
  }

  public double getAngle() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  public boolean atAngle(double angle) {
    return atAngle(angle, 1);
  }

  private boolean atAngle(double angle, double tolerance) {
    return Math.abs(angle - getAngle()) < tolerance;
  }

  public boolean atAngleForSpeaker(double distance) {
    return atAngle(getAngleFromDistanceToSpeaker(distance), getToleranceFromDistance(distance));
  }

  public boolean atAngleForFloorSpot(double distance) {
    return atAngle(getAngleFromDistanceToFloorSpot(distance), 5);
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public double getAngleFromDistanceToSpeaker(double distance) {
    return speakerDistanceToAngle.get(distance);
  }

  private double getToleranceFromDistance(double distance) {
    return distance > 8 ? 0.5 : distance < 0.85 ? 5.0 : (distanceToAngleTolerance.get(distance));
  }

  public double getAngleFromDistanceToFloorSpot(double distance) {
    return floorSpotDistanceToAngle.get(distance);
  }
}

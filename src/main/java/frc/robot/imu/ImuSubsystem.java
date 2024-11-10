// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class ImuSubsystem extends LifecycleSubsystem {
  private final Pigeon2 imu;
  private final InterpolatingDoubleTreeMap distanceToAngleTolerance =
      new InterpolatingDoubleTreeMap();
  private final TimeInterpolatableBuffer<Double> robotHeadingHistory =
      TimeInterpolatableBuffer.createDoubleBuffer(3);

  public ImuSubsystem(SwerveSubsystem swerve) {
    super(SubsystemPriority.IMU);

    this.imu = swerve.drivetrainPigeon;

    RobotConfig.get().imu().distanceToAngleTolerance().accept(distanceToAngleTolerance);

    Pigeon2Configuration config =
        new Pigeon2Configuration()
            .withMountPose(
                new MountPoseConfigs()
                    .withMountPosePitch(0)
                    .withMountPoseRoll(0)
                    .withMountPoseYaw(0));

    imu.getConfigurator().apply(config);
  }

  @Override
  public void robotPeriodic() {
    double robotHeading = this.getRobotHeading();
    DogLog.log("Imu/RobotHeading", robotHeading);
    DogLog.log("Imu/RobotHeadingModulo", MathUtil.inputModulus(robotHeading, 0, 360));
    DogLog.log("Imu/RobotHeadingRadians", Units.degreesToRadians(robotHeading));

    var yaw = this.imu.getYaw();

    double yawOffset = Utils.getCurrentTimeSeconds() - yaw.getTimestamp().getTime();

    robotHeadingHistory.addSample(
        Timer.getFPGATimestamp() - yawOffset, Units.degreesToRadians(yaw.getValueAsDouble()));
  }

  public double getRobotHeading() {
    return MathUtil.inputModulus(imu.getYaw().getValueAsDouble(), -180, 180);
  }

  public double getRobotHeading(double timestamp) {
    return Units.radiansToDegrees(
        robotHeadingHistory
            .getSample(timestamp)
            .orElseGet(() -> Units.degreesToRadians(getRobotHeading())));
  }

  public double getPitch() {
    return imu.getPitch().getValueAsDouble();
  }

  public double getPitchRate() {
    return imu.getAngularVelocityYWorld().getValueAsDouble();
  }

  public double getRoll() {
    return imu.getRoll().getValueAsDouble();
  }

  public double getRollRate() {
    return imu.getAngularVelocityXWorld().getValueAsDouble();
  }

  public double getRobotAngularVelocity() {
    return imu.getRate();
  }

  public void setAngle(double zeroAngle) {
    this.imu.setYaw(zeroAngle);
  }

  private boolean atAngle(double angle, double tolerance) {
    // Probably redundant to normalize the angle here but I don't want to check
    var normalizedAngle = MathUtil.inputModulus(angle, -180, 180);

    return MathUtil.isNear(normalizedAngle, getRobotHeading(), tolerance, -180, 180);
  }

  public boolean belowVelocityForVision(double distance) {
    return getRobotAngularVelocity() < 10;
  }

  public boolean atAngleForSpeaker(double angle, double distance) {
    return atAngle(angle, 2.5);
  }

  public boolean atAngleForFloorSpot(double angle) {
    return atAngle(angle, 10);
  }

  public double getYAcceleration() {
    return imu.getAccelerationY().getValueAsDouble();
  }

  public double getXAcceleration() {
    return imu.getAccelerationX().getValueAsDouble();
  }
}

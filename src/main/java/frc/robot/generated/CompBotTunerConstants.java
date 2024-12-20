// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class CompBotTunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(110).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3.5).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 40.0;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  public static final double kSpeedAt12VoltsMps = 5.96;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 3.125;

  private static final double kDriveGearRatio = (50.0 / 14.0) / (28.0 / 16.0) / (15.0 / 45.0);
  private static final double kSteerGearRatio = 21.428571428571427;
  private static final double kWheelRadiusInches = 2;

  private static final boolean kSteerMotorReversed = true;
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final String kCANbusName = "581CANivore";
  private static final int kPigeonId = 1;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;
  // Simulated voltage necessary to overcome friction
  private static final double kSteerFrictionVoltage = 0.25;
  private static final double kDriveFrictionVoltage = 0.25;

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

  public static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorInitialConfigs(
              new TalonFXConfiguration()
                  // DO NOT use the existing object, the CTR Swerve Generator will mutate values in
                  // it. Check RobotConfig for the actual used current limits.
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(50)
                          .withStatorCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true)
                          .withStatorCurrentLimitEnable(true))
                  // DO NOT use the existing object, the CTR Swerve Generator will mutate values in
                  // it. Check RobotConfig for the actual used current limits.
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(80)
                          .withPeakReverseTorqueCurrent(-80)))
          .withSteerMotorInitialConfigs(
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(20)
                          .withStatorCurrentLimit(70)
                          .withSupplyCurrentLimitEnable(true)
                          .withStatorCurrentLimitEnable(true)))
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withSteerMotorInverted(kSteerMotorReversed);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 2;
  private static final int kFrontLeftSteerMotorId = 3;
  private static final int kFrontLeftEncoderId = 10;
  private static final double kFrontLeftEncoderOffset = -0.486083984375;

  private static final double kFrontLeftXPosInches = 10.125;
  private static final double kFrontLeftYPosInches = 11.375;

  // Front Right
  private static final int kFrontRightDriveMotorId = 4;
  private static final int kFrontRightSteerMotorId = 5;
  private static final int kFrontRightEncoderId = 11;
  private static final double kFrontRightEncoderOffset = -0.27734375;

  private static final double kFrontRightXPosInches = 10.125;
  private static final double kFrontRightYPosInches = -11.375;

  // Back Left
  private static final int kBackLeftDriveMotorId = 6;
  private static final int kBackLeftSteerMotorId = 7;
  private static final int kBackLeftEncoderId = 12;
  private static final double kBackLeftEncoderOffset = 0.302978515625;

  private static final double kBackLeftXPosInches = -10.125;
  private static final double kBackLeftYPosInches = 11.375;

  // Back Right
  private static final int kBackRightDriveMotorId = 8;
  private static final int kBackRightSteerMotorId = 9;
  private static final int kBackRightEncoderId = 13;
  private static final double kBackRightEncoderOffset = -0.19287109375;

  private static final double kBackRightXPosInches = -10.125;
  private static final double kBackRightYPosInches = -11.375;

  public static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          Units.inchesToMeters(kFrontLeftXPosInches),
          Units.inchesToMeters(kFrontLeftYPosInches),
          kInvertLeftSide);
  public static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          Units.inchesToMeters(kFrontRightXPosInches),
          Units.inchesToMeters(kFrontRightYPosInches),
          kInvertRightSide);
  public static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          Units.inchesToMeters(kBackLeftXPosInches),
          Units.inchesToMeters(kBackLeftYPosInches),
          kInvertLeftSide);
  public static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          Units.inchesToMeters(kBackRightXPosInches),
          Units.inchesToMeters(kBackRightYPosInches),
          kInvertRightSide);
}

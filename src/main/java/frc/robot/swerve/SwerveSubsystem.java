// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class SwerveSubsystem extends LifecycleSubsystem {
  private static final double MAX_SPEED_SHOOTING = Units.feetToMeters(0.5);
  private static final double MAX_FLOOR_SPEED_SHOOTING = Units.feetToMeters(18);
  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);
  private boolean isShooting = false;

  private double leftXDeadband = 0.05;
  private double rightXDeadband = 0.05;
  private double leftYDeadband = 0.05;

  public static final Translation2d FRONT_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontLeft.LocationX, CommandSwerveDrivetrain.FrontLeft.LocationY);
  public static final Translation2d FRONT_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontRight.LocationX,
          CommandSwerveDrivetrain.FrontRight.LocationY);
  public static final Translation2d BACK_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackLeft.LocationX, CommandSwerveDrivetrain.BackLeft.LocationY);
  public static final Translation2d BACK_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackRight.LocationX, CommandSwerveDrivetrain.BackRight.LocationY);
  public static final Translation2d[] MODULE_LOCATIONS =
      new Translation2d[] {
        FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION
      };
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_LOCATIONS);
  private final double TimeConstant = 0.2;
  private final double AccelerationLimit = 3.2;
  private Translation2d previousVelocity = new Translation2d();

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private boolean snapToAngle = false;
  private Rotation2d goalSnapAngle = new Rotation2d();
  private final CommandXboxController controller;

  // My drivetrain
  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();

  public final Pigeon2 drivetrainPigeon = drivetrain.getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);
  private final SwerveModule frontLeft = drivetrain.getModule(0);
  private final SwerveModule frontRight = drivetrain.getModule(1);
  private final SwerveModule backLeft = drivetrain.getModule(2);
  private final SwerveModule backRight = drivetrain.getModule(3);
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private boolean closedLoop = false;

  private final PIDController xPid = new PIDController(3.2, 0, 0);
  private final PIDController yPid = new PIDController(3.2, 0, 0);
  private final PIDController omegaPid = new PIDController(1.0, 0, 0);

  public SwerveSubsystem(CommandXboxController driveController) {
    super(SubsystemPriority.SWERVE);
    this.controller = driveController;

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    driveToAngle.HeadingController = RobotConfig.get().swerve().snapController();
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveToAngle.HeadingController.setTolerance(0.02);

    omegaPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public List<SwerveModulePosition> getModulePositions() {
    return List.of(
        frontLeft.getPosition(true),
        frontRight.getPosition(true),
        backLeft.getPosition(true),
        backRight.getPosition(true));
  }

  public void setSnapToAngle(Rotation2d angle) {
    goalSnapAngle = angle;
    snapToAngle = true;
  }

  public void disableSnapToAngle() {
    snapToAngle = false;
  }

  public void setFieldRelativeSpeeds(ChassisSpeeds speeds, boolean closedLoop) {
    this.fieldRelativeSpeeds = speeds;
    this.closedLoop = closedLoop;
    // Send a swerve request each time new chassis speeds are provided
    sendSwerveRequest();
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds, boolean closedLoop) {
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drivetrain.getState().Pose.getRotation());
    setFieldRelativeSpeeds(fieldRelative, closedLoop);
  }

  public Command driveTeleopCommand() {
    return run(() -> {
          if (!DriverStation.isTeleop()) {
            return;
          }
          double leftY =
              -1.0
                  * ControllerHelpers.getExponent(
                      ControllerHelpers.getDeadbanded(controller.getLeftY(), leftYDeadband), 1.5);
          double leftX =
              ControllerHelpers.getExponent(
                  ControllerHelpers.getDeadbanded(controller.getLeftX(), leftXDeadband), 1.5);
          double rightX =
              ControllerHelpers.getExponent(
                  ControllerHelpers.getDeadbanded(controller.getRightX(), rightXDeadband), 2);

          if (RobotConfig.get().swerve().invertRotation()) {
            rightX *= -1.0;
          }

          if (RobotConfig.get().swerve().invertX()) {
            leftX *= -1.0;
          }

          if (RobotConfig.get().swerve().invertY()) {
            leftY *= -1.0;
          }

          if (FmsSubsystem.isRedAlliance()) {
            leftX *= -1.0;
            leftY *= -1.0;
          }

          ChassisSpeeds teleopSpeeds =
              new ChassisSpeeds(
                  -1.0 * leftY * MaxSpeed,
                  leftX * MaxSpeed,
                  rightX * TELEOP_MAX_ANGULAR_RATE.getRadians());

          DogLog.log("Swerve/RawTeleopSpeeds", teleopSpeeds);

          double currentSpeed =
              Math.sqrt(
                  Math.pow(teleopSpeeds.vxMetersPerSecond, 2)
                      + Math.pow(teleopSpeeds.vyMetersPerSecond, 2));
          DogLog.log("Swerve/CurrentSpeed", currentSpeed);
          var scaled = teleopSpeeds.div(currentSpeed / MAX_SPEED_SHOOTING);
          DogLog.log("Swerve/ScaledSpeeds", scaled);
          if (isShooting) {
            if (currentSpeed > MAX_SPEED_SHOOTING) {
              teleopSpeeds =
                  new ChassisSpeeds(
                      scaled.vxMetersPerSecond,
                      scaled.vyMetersPerSecond,
                      teleopSpeeds.omegaRadiansPerSecond);
            }
          }

          DogLog.log("Swerve/UsedTeleopSpeeds", teleopSpeeds);

          setFieldRelativeSpeeds(teleopSpeeds, false);
        })
        .withName("DriveTeleopCommand");
  }

  private ChassisSpeeds accelerationLimitChassisSpeeds(ChassisSpeeds speeds) {

    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double maxVelocitychange = AccelerationLimit * TimeConstant;

    Translation2d velocityChange = (velocity.minus(previousVelocity));
    double velocityChangeAngle =
        Math.atan2(velocityChange.getY(), velocityChange.getX()); // Radians
    Translation2d limitedVelocityVectorChange = velocityChange;
    Translation2d limitedVelocityVector = velocity;

    double acceleration = velocity.getNorm() - previousVelocity.getNorm();
    DogLog.log("Swerve/Acceleration", acceleration);

    if (velocityChange.getNorm() > maxVelocitychange && acceleration > 0) {
      limitedVelocityVectorChange =
          new Translation2d(maxVelocitychange, new Rotation2d(velocityChangeAngle));
      limitedVelocityVector = previousVelocity.plus(limitedVelocityVectorChange);
    }

    previousVelocity = limitedVelocityVector;

    return new ChassisSpeeds(
        limitedVelocityVector.getX(), limitedVelocityVector.getY(), speeds.omegaRadiansPerSecond);
  }

  public void setShootingMode(boolean value) {
    isShooting = value;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return KINEMATICS.toChassisSpeeds(drivetrain.getState().ModuleStates);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getRobotRelativeSpeeds(),
        Rotation2d.fromDegrees(drivetrainPigeon.getYaw().getValueAsDouble()));
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("Swerve/SnapToAngle", snapToAngle);
    DogLog.log("Swerve/SnapToAngleGoal", goalSnapAngle.getDegrees());
    DogLog.log("Swerve/Pose", drivetrain.getState().Pose);
    DogLog.log("Swerve/ModuleStates", drivetrain.getState().ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrain.getState().ModuleTargets);

    DogLog.log("Swerve/RobotSpeed", getRobotRelativeSpeeds());

    // Send a swerve request at least once every loop
    sendSwerveRequest();
  }

  private void sendSwerveRequest() {
    DriveRequestType driveType;

    if (closedLoop) {
      driveType = DriveRequestType.Velocity;
    } else {
      driveType = DriveRequestType.OpenLoopVoltage;
    }

    var limitedSpeeds = accelerationLimitChassisSpeeds(fieldRelativeSpeeds);

    if (snapToAngle) {
      drivetrain.setControl(
          driveToAngle
              .withVelocityX(limitedSpeeds.vxMetersPerSecond)
              .withVelocityY(limitedSpeeds.vyMetersPerSecond)
              .withTargetDirection(goalSnapAngle)
              .withDriveRequestType(driveType));
    } else {
      drivetrain.setControl(
          drive
              .withVelocityX(limitedSpeeds.vxMetersPerSecond)
              .withVelocityY(limitedSpeeds.vyMetersPerSecond)
              .withRotationalRate(limitedSpeeds.omegaRadiansPerSecond)
              .withDriveRequestType(driveType));
    }
  }

  public boolean snapsEnabled() {
    return snapToAngle;
  }

  public Rotation2d snapAngle() {
    return goalSnapAngle;
  }

  public boolean movingSlowEnoughForSpeakerShot() {
    return movingSlowEnoughForSpeakerShot(getRobotRelativeSpeeds());
  }

  public boolean movingSlowEnoughForFloorShot() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_FLOOR_SPEED_SHOOTING;
  }

  public boolean movingSlowEnoughForSpeakerShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_SPEED_SHOOTING;
  }

  private boolean atLocation(Pose2d target, Pose2d current) {
    // Return true once at location
    double translationTolerance = 0.1;
    double omegaTolerance = 5;
    return Math.abs(current.getX() - target.getX()) <= translationTolerance
        && Math.abs(current.getY() - target.getY()) <= translationTolerance
        && Math.abs(current.getRotation().getDegrees() - target.getRotation().getDegrees())
            <= omegaTolerance;
  }

  public Command driveToPoseCommand(
      Supplier<Optional<Pose2d>> targetSupplier, Supplier<Pose2d> currentPose, boolean shouldEnd) {
    return run(() -> {
          var maybeTarget = targetSupplier.get();

          if (!maybeTarget.isPresent()) {
            setFieldRelativeSpeeds(new ChassisSpeeds(), closedLoop);
            DogLog.log("Debug/DriveToPoseNoTarget", Timer.getFPGATimestamp());
            return;
          }

          DogLog.log("Debug/DriveToPoseHasTarget", Timer.getFPGATimestamp());

          var target = maybeTarget.get();

          var pose = currentPose.get();
          double vx = xPid.calculate(pose.getX(), target.getX());
          double vy = yPid.calculate(pose.getY(), target.getY());

          var newSpeeds = new ChassisSpeeds(vx, vy, 0);
          setFieldRelativeSpeeds(newSpeeds, true);
        })
        .until(
            () -> {
              if (shouldEnd) {
                var maybeTarget = targetSupplier.get();
                if (maybeTarget.isPresent()) {
                  var target = targetSupplier.get();

                  return atLocation(target.get(), currentPose.get());
                }
              }

              return false;
            })
        .finallyDo(
            () -> {
              snapToAngle = false;
            });
  }
}

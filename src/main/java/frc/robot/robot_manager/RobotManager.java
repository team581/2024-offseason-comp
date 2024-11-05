// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.climber.ClimberMode;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorPositions;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_manager.NoteManager;
import frc.robot.note_manager.NoteState;
import frc.robot.shooter.ShooterMode;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;
import frc.robot.wrist.WristPositions;
import frc.robot.wrist.WristSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final WristSubsystem wrist;
  public final ElevatorSubsystem elevator;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final ClimberSubsystem climber;
  public final SwerveSubsystem swerve;
  public final SnapManager snaps;
  private final ImuSubsystem imu;
  public final NoteManager noteManager;

  private DistanceAngle fieldRelativeDistanceAngleToSpeaker = new DistanceAngle(0, 0, false);
  private DistanceAngle fieldRelativeDistanceAngleToFeedSpot = new DistanceAngle(0, 0, false);
  private double wristAngleForSpeaker = 0.0;
  private double wristAngleForFloorSpot = 0.0;

  private final Timer dropNoteSensorDebounce = new Timer();
  public final double DROP_DEBOUNCE_TIME_SECONDS = 0.4;

  public RobotManager(
      WristSubsystem wrist,
      ElevatorSubsystem elevator,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      ClimberSubsystem climber,
      SwerveSubsystem swerve,
      SnapManager snaps,
      ImuSubsystem imu,
      NoteManager noteManager) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.wrist = wrist;
    this.elevator = elevator;
    this.shooter = shooter;
    this.localization = localization;
    this.vision = vision;
    this.climber = climber;
    this.swerve = swerve;
    this.snaps = snaps;
    this.imu = imu;
    this.noteManager = noteManager;
  }

  @Override
  protected void collectInputs() {
    fieldRelativeDistanceAngleToSpeaker = vision.getDistanceAngleSpeaker();
    fieldRelativeDistanceAngleToFeedSpot = vision.getDistanceAngleFloorShot();
    wristAngleForSpeaker =
        wrist.getAngleFromDistanceToSpeaker(fieldRelativeDistanceAngleToSpeaker.distance());
    wristAngleForFloorSpot =
        wrist.getAngleFromDistanceToFloorSpot(fieldRelativeDistanceAngleToFeedSpot.distance());
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_WITH_GP,
              WAITING_SPEAKER_SHOT,
              WAITING_AMP_SHOT,
              WAITING_FLOOR_SHOT,
              WAITING_DROP,
              WAITING_SUBWOOFER_SHOT,
              WAITING_PODIUM_SHOT,
              WAITING_SHOOTER_AMP,
              OUTTAKING,
              UNJAM,
              SHOOTER_STOPPED_UNJAM,
              CLIMB_1_LINEUP_OUTER,
              CLIMB_2_LINEUP_INNER,
              CLIMB_3_LINEUP_FINAL,
              CLIMB_4_HANGING,
              CLIMB_6_HANGING_FINISHED,
              PREPARE_PRESET_3,
              PREPARE_PRESET_AMP,
              PREPARE_PRESET_SOURCE,
              PREPARE_PRESET_MIDDLE ->
          currentState;
      case PREPARE_WAITING_AMP_SHOT ->
          (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
                  && wrist.atAngle(WristPositions.FULLY_STOWED))
              ? RobotState.WAITING_AMP_SHOT
              : currentState;
      case INTAKING ->
          (noteManager.getState() == NoteState.INTAKE_TO_QUEUER
                  ||

                  // Already have a note
                  noteManager.getState() == NoteState.IDLE_IN_QUEUER)
              ? RobotState.FINISH_INTAKING
              : currentState;

      case FINISH_INTAKING, LAZY_INTAKING ->
          (noteManager.getState() == NoteState.IDLE_IN_QUEUER)
              ? RobotState.IDLE_WITH_GP
              : currentState;
      case PREPARE_FLOOR_SHOT -> {
        var wristAtGoal =
            wrist.atAngleForFloorSpot(fieldRelativeDistanceAngleToFeedSpot.distance());
        var shooterAtGoal = shooter.atGoal(ShooterMode.FLOOR_SHOT);
        var headingAtGoal =
            imu.atAngleForFloorSpot(fieldRelativeDistanceAngleToFeedSpot.targetAngle());
        var swerveAtGoal = swerve.movingSlowEnoughForFloorShot();
        var angularVelocityAtGoal = Math.abs(imu.getRobotAngularVelocity()) < 360.0;
        DogLog.log("RobotManager/FloorShot/WristAtGoal", wristAtGoal);
        DogLog.log("RobotManager/FloorShot/ShooterAtGoal", shooterAtGoal);
        DogLog.log("RobotManager/FloorShot/HeadingAtGoal", headingAtGoal);
        DogLog.log("RobotManager/FloorShot/SwerveAtGoal", swerveAtGoal);
        DogLog.log("RobotManager/FloorShot/AngularVelocityAtGoal", angularVelocityAtGoal);

        if (wristAtGoal
            && shooterAtGoal
            && headingAtGoal
            // && jitterAtGoal
            && swerveAtGoal
            && angularVelocityAtGoal) {
          yield RobotState.FLOOR_SHOOT;
        }
        yield currentState;
      }
      case PREPARE_DROPPING -> {
        var shooterAtDropSpeed = shooter.atGoal(ShooterMode.DROPPING);
        if (shooterAtDropSpeed) {
          yield RobotState.DROPPING;
        }
        yield currentState;
      }
      case PREPARE_PODIUM_SHOT ->
          (wrist.atAngle(WristPositions.PODIUM_SHOT)
                  && shooter.atGoal(ShooterMode.PODIUM_SHOT)
                  && noteManager.getState() == NoteState.IDLE_IN_QUEUER)
              ? RobotState.PODIUM_SHOOT
              : currentState;
      case PREPARE_SUBWOOFER_SHOT ->
          (wrist.atAngle(WristPositions.SUBWOOFER_SHOT)
                  && shooter.atGoal(ShooterMode.SUBWOOFER_SHOT)
                  && noteManager.getState() == NoteState.IDLE_IN_QUEUER)
              ? RobotState.SUBWOOFER_SHOOT
              : currentState;
      case PREPARE_AMP_SHOT ->
          (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
                  && wrist.atAngle(WristPositions.STOWED))
              ? RobotState.AMP_SHOT
              : currentState;
      case PREPARE_SHOOTER_AMP ->
          (noteManager.getState() == NoteState.IDLE_IN_QUEUER
                  && wrist.atAngle(WristPositions.SHOOTER_AMP)
                  && shooter.atGoal(ShooterMode.SHOOTER_AMP)
                  && elevator.atPosition(ElevatorPositions.STOWED))
              ? RobotState.SHOOTER_AMP
              : currentState;
      case PREPARE_SPEAKER_SHOT -> {
        boolean wristAtGoal =
            wrist.atAngleForSpeaker(fieldRelativeDistanceAngleToSpeaker.distance());
        boolean shooterAtGoal = shooter.atGoal(ShooterMode.SPEAKER_SHOT);
        boolean swerveSlowEnough = swerve.movingSlowEnoughForSpeakerShot();
        boolean angularVelocitySlowEnough =
            imu.belowVelocityForVision(fieldRelativeDistanceAngleToSpeaker.distance());
        boolean robotHeadingAtGoal =
            imu.atAngleForSpeaker(
                fieldRelativeDistanceAngleToSpeaker.targetAngle(),
                fieldRelativeDistanceAngleToSpeaker.distance());
        boolean limelightWorking = false;

        if (DriverStation.isAutonomous() && vision.getState() == VisionState.OFFLINE) {
          limelightWorking = true;
        } else {
          limelightWorking = vision.getState() == VisionState.SEES_TAGS;
        }

        DogLog.log("RobotManager/SpeakerShot/LimelightWorking", limelightWorking);
        DogLog.log("RobotManager/SpeakerShot/WristAtGoal", wristAtGoal);
        DogLog.log("RobotManager/SpeakerShot/ShooterAtGoal", shooterAtGoal);
        DogLog.log("RobotManager/SpeakerShot/SwerveSlowEnough", swerveSlowEnough);
        DogLog.log("RobotManager/SpeakerShot/AngularVelocitySlowEnough", angularVelocitySlowEnough);
        DogLog.log("RobotManager/SpeakerShot/RobotHeadingAtGoal", robotHeadingAtGoal);
        if (limelightWorking
            && wristAtGoal
            && shooterAtGoal
            && swerveSlowEnough
            && angularVelocitySlowEnough
            && robotHeadingAtGoal) {
          yield RobotState.SPEAKER_SHOOT;
        }
        yield currentState;
      }
      case PREPARE_PASS_LOW ->
          (noteManager.getState() == NoteState.IDLE_IN_QUEUER
                  && shooter.atGoal(ShooterMode.OUTTAKE))
              ? RobotState.PASS_LOW
              : currentState;
      case WAITING_MULTI_SPEAKER_SHOT ->
          (noteManager.getState() == NoteState.IDLE_IN_QUEUER)
              ? RobotState.PREPARE_SPEAKER_SHOT
              : currentState;
      case WAITING_MULTI_FLOOR_SHOT ->
          (noteManager.getState() == NoteState.IDLE_IN_QUEUER)
              ? RobotState.PREPARE_FLOOR_SHOT
              : currentState;
      case PASS_LOW,
              SHOOTER_AMP,
              SUBWOOFER_SHOOT,
              PODIUM_SHOOT,
              AMP_SHOT,
              PRESET_3,
              PRESET_MIDDLE,
              PRESET_AMP ->
          (noteManager.getState() == NoteState.IDLE_NO_GP) ? RobotState.IDLE_NO_GP : currentState;

      case DROPPING -> {
        if (noteManager.getState() == NoteState.IDLE_NO_GP
            && dropNoteSensorDebounce.hasElapsed(DROP_DEBOUNCE_TIME_SECONDS)) {
          dropNoteSensorDebounce.reset();
          yield RobotState.IDLE_NO_GP;
        }
        yield currentState;
      }
      case PRESET_LEFT ->
          (noteManager.getState() == NoteState.IDLE_NO_GP) ? RobotState.IDLE_NO_GP : currentState;
      case FLOOR_SHOOT ->
          (noteManager.getState() == NoteState.IDLE_NO_GP)
              ? RobotState.WAITING_MULTI_FLOOR_SHOT
              : currentState;
      case SPEAKER_SHOOT -> {
        if (noteManager.getState()
                == NoteState
                    .IDLE_NO_GP // TODO: Super hacky workaround for not being able to exit after
            // multi speaker shot
            || !(noteManager.intake.hasNote() && noteManager.queuer.hasNote())) {
          if (DriverStation.isAutonomous()) {
            yield RobotState.IDLE_NO_GP;
          } else {
            yield RobotState.WAITING_MULTI_SPEAKER_SHOT;
          }
        }
        yield currentState;
      }

      case PREPARE_CLIMB_4_HANGING ->
          (climber.atGoal(ClimberMode.HANGING) && elevator.atPosition(ElevatorPositions.CLIMBING))
              ? RobotState.CLIMB_4_HANGING
              : currentState;
      case PREPARE_CLIMB_5_HANGING_TRAP_SCORE ->
          (elevator.atPosition(ElevatorPositions.TRAP_SHOT) && climber.atGoal(ClimberMode.HANGING))
              ? RobotState.CLIMB_5_HANGING_TRAP_SCORE
              : currentState;
      case CLIMB_5_HANGING_TRAP_SCORE ->
          (noteManager.getState() == NoteState.IDLE_NO_GP)
              ? RobotState.CLIMB_4_HANGING
              : currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleNoGPRequest();
      }
      case IDLE_WITH_GP -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
      }
      case LAZY_INTAKING -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.lazyIntakeRequest();
      }
      case FINISH_INTAKING, INTAKING -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
      }
      case OUTTAKING -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.outtakeRequest();
      }
      case PREPARE_PASS_LOW -> {
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
      }
      case PASS_LOW -> {
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterOuttakeRequest();
      }
      case PREPARE_DROPPING -> {
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.DROPPING);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        dropNoteSensorDebounce.reset();
        dropNoteSensorDebounce.stop();
      }
      case DROPPING -> {
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.DROPPING);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.dropRequest();
        dropNoteSensorDebounce.start();
      }
      case WAITING_DROP -> {
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.DROPPING);
        climber.setGoalMode(ClimberMode.STOWED);
        dropNoteSensorDebounce.reset();
        dropNoteSensorDebounce.stop();
      }
      case PREPARE_SHOOTER_AMP, WAITING_SHOOTER_AMP -> {
        wrist.setAngle(WristPositions.SHOOTER_AMP);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SHOOTER_AMP);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
      }
      case SHOOTER_AMP -> {
        wrist.setAngle(WristPositions.SHOOTER_AMP);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SHOOTER_AMP);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
      }
      case WAITING_FLOOR_SHOT, PREPARE_FLOOR_SHOT -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();

        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case FLOOR_SHOOT -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();

        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case WAITING_PODIUM_SHOT, PREPARE_PODIUM_SHOT -> {
        wrist.setAngle(WristPositions.PODIUM_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PODIUM_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        snaps.setAngle(SnapManager.getPodiumAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PODIUM_SHOOT -> {
        wrist.setAngle(WristPositions.PODIUM_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PODIUM_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(SnapManager.getPodiumAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case WAITING_SUBWOOFER_SHOT, PREPARE_SUBWOOFER_SHOT -> {
        if (DriverStation.isAutonomous()) {
          wrist.setAngle(WristPositions.AUTO_SUBWOOFER_SHOT);
        } else {
          wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        }
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
      }
      case SUBWOOFER_SHOOT -> {
        if (DriverStation.isAutonomous()) {
          wrist.setAngle(WristPositions.AUTO_SUBWOOFER_SHOT);
        } else {
          wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        }
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
      }
      case WAITING_SPEAKER_SHOT, PREPARE_SPEAKER_SHOT -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();

        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case SPEAKER_SHOOT -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();

        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PREPARE_WAITING_AMP_SHOT -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.ampWaitRequest();
      }
      case WAITING_AMP_SHOT, PREPARE_AMP_SHOT -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.ampWaitRequest();
      }
      case AMP_SHOT -> {
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.ampScoreRequest();
      }
      case UNJAM -> {
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.ANTI_JAM);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.unjamRequest();
      }
      case SHOOTER_STOPPED_UNJAM -> {
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        elevator.setGoalHeight(ElevatorPositions.ANTI_JAM);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.unjamRequest();
      }
      case CLIMB_1_LINEUP_OUTER -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_OUTER);
        noteManager.trapWaitRequest();
      }
      case CLIMB_2_LINEUP_INNER -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_INNER);
        noteManager.trapWaitRequest();
      }
      case CLIMB_3_LINEUP_FINAL -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_INNER);
        noteManager.trapWaitRequest();
      }
      case PREPARE_CLIMB_4_HANGING, CLIMB_4_HANGING -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapWaitRequest();
      }
      case PREPARE_CLIMB_5_HANGING_TRAP_SCORE -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapWaitRequest();
      }
      case CLIMB_5_HANGING_TRAP_SCORE -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        elevator.setPulsing(false);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapShotRequest();
        // No matter what, try doing the trap score
        // If the handoff isn't completed at this point, there's no way to finish it while the
        // elevator is up
        // So, just try scoring regardless of robot state
        noteManager.evilStateOverride(NoteState.TRAP_SCORING);
      }
      case CLIMB_6_HANGING_FINISHED -> {
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING_FINISHED);
        elevator.setPulsing(false);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.idleNoGPRequest();
      }
      case PRESET_AMP -> {
        wrist.setAngle(WristPositions.PRESET_AMP);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_RIGHT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(SnapManager.getPresetAmpAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PRESET_LEFT -> {
        wrist.setAngle(WristPositions.PRESET_SOURCE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_SOURCE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(vision.getDistanceAngleSpeaker().targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PRESET_MIDDLE -> {
        wrist.setAngle(WristPositions.PRESET_MIDDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_MIDDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(vision.getDistanceAngleSpeaker().targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PRESET_3 -> {
        wrist.setAngle(WristPositions.PRESET_3);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_3);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(vision.getDistanceAngleSpeaker().targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PREPARE_PRESET_3 -> {
        wrist.setAngle(WristPositions.PRESET_3);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_3);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
        snaps.setAngle(vision.getDistanceAngleSpeaker().targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PREPARE_PRESET_AMP -> {
        wrist.setAngle(WristPositions.PRESET_AMP);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_RIGHT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
        snaps.setAngle(SnapManager.getPresetAmpAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PREPARE_PRESET_SOURCE -> {
        wrist.setAngle(WristPositions.PRESET_SOURCE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_SOURCE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
        snaps.setAngle(vision.getDistanceAngleSpeaker().targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PREPARE_PRESET_MIDDLE -> {
        wrist.setAngle(WristPositions.PRESET_MIDDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PRESET_MIDDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
        snaps.setAngle(vision.getDistanceAngleSpeaker().targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case WAITING_MULTI_SPEAKER_SHOT -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
      }
      case WAITING_MULTI_FLOOR_SHOT -> {
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
      }
    }
    swerve.setShootingMode(newState.shootingMode);
    SmartDashboard.putBoolean("HasNote", getState().hasNote);
    DogLog.log("NoteMapManager/DropNoteTimerValue", dropNoteSensorDebounce.get());
  }

  @Override
  public void robotPeriodic() {
    // TODO: Remove when done tuning shots
    DogLog.log("Debug/FeedDistance", fieldRelativeDistanceAngleToFeedSpot.distance());
    switch (getState()) {
      case WAITING_SPEAKER_SHOT, PREPARE_SPEAKER_SHOT, SPEAKER_SHOOT -> {
        snaps.setAngle(fieldRelativeDistanceAngleToSpeaker.targetAngle());
        wrist.setAngle(wristAngleForSpeaker);
      }
      case IDLE_NO_GP,
          IDLE_WITH_GP,
          LAZY_INTAKING,
          FINISH_INTAKING,
          INTAKING,
          OUTTAKING,
          WAITING_MULTI_SPEAKER_SHOT -> {
        wrist.setAngle(wristAngleForSpeaker);
      }
      case WAITING_FLOOR_SHOT, PREPARE_FLOOR_SHOT, FLOOR_SHOOT, WAITING_MULTI_FLOOR_SHOT -> {
        snaps.setAngle(fieldRelativeDistanceAngleToFeedSpot.targetAngle());
        wrist.setAngle(wristAngleForFloorSpot);
      }
    }
  }

  public void waitPodiumShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.WAITING_PODIUM_SHOT);
    }
  }

  public void podiumShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_PODIUM_SHOT);
    }
  }

  public void waitSubwooferShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.WAITING_SUBWOOFER_SHOT);
    }
  }

  public void subwooferShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_SUBWOOFER_SHOT);
    }
  }

  public void waitSpeakerShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.WAITING_SPEAKER_SHOT);
    }
  }

  public void speakerShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_SPEAKER_SHOT);
    }
  }

  public void forceSpeakerShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.SPEAKER_SHOOT);
    }
  }

  public void waitAmpShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.WAITING_AMP_SHOT);
    }
  }

  public void ampShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_AMP_SHOT);
    }
  }

  public void waitShooterAmpRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.WAITING_SHOOTER_AMP);
    }
  }

  public void shooterAmpRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_SHOOTER_AMP);
    }
  }

  public void waitingDropRequest() {
    // Prevent command from instantly ending if robot doesn't think it has a note
    noteManager.evilStateOverride(NoteState.IDLE_IN_QUEUER);
    if (!getState().climbing) {
      setStateFromRequest(RobotState.WAITING_DROP);
    }
  }

  public void dropRequest() {
    // Prevent command from instantly ending if robot doesn't think it has a note
    noteManager.evilStateOverride(NoteState.IDLE_IN_QUEUER);
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_DROPPING);
    }
  }

  public void preparePresetRightRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_PRESET_AMP);
    }
  }

  public void preparePresetLeftRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_PRESET_SOURCE);
    }
  }

  public void preparePresetMiddleRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_PRESET_MIDDLE);
    }
  }

  public void preparePreset3Request() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_PRESET_3);
    }
  }

  public void presetRightRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PRESET_AMP);
    }
  }

  public void presetLeftRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PRESET_LEFT);
    }
  }

  public void presetMiddleRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PRESET_MIDDLE);
    }
  }

  public void preset3Request() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PRESET_3);
    }
  }

  public void waitFloorShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.WAITING_FLOOR_SHOT);
    }
  }

  public void floorShotRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_FLOOR_SHOT);
    }
  }

  public void intakeRequest() {
    if (!getState().climbing) {
      // Reset note manager state so that we don't instantly think we're done intaking
      // Need to force set the state, rather than doing a state request, due to order of
      // subsystems executing
      noteManager.evilStateOverride(NoteState.IDLE_NO_GP);
      setStateFromRequest(RobotState.INTAKING);
    }
  }

  public void outtakeRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.OUTTAKING);
    }
  }

  public void outtakeShooterRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.PREPARE_PASS_LOW);
    }
  }

  public void stowRequest() {
    if (getState().hasNote) {
      switch (getState()) {
        case WAITING_AMP_SHOT -> setStateFromRequest(RobotState.AMP_SHOT);
        default -> setStateFromRequest(RobotState.IDLE_WITH_GP);
      }
    } else {
      setStateFromRequest(RobotState.IDLE_NO_GP);
    }
  }

  public void idleNoGPRequest() {
    setStateFromRequest(RobotState.IDLE_NO_GP);
  }

  public void unjamRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void shooterStoppedUnjamRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.SHOOTER_STOPPED_UNJAM);
    }
  }

  public void stopShootingRequest() {

    switch (getState()) {
      case PREPARE_PODIUM_SHOT, WAITING_PODIUM_SHOT -> waitPodiumShotRequest();
      case PREPARE_FLOOR_SHOT, WAITING_FLOOR_SHOT -> waitFloorShotRequest();
      case PREPARE_SUBWOOFER_SHOT, WAITING_SUBWOOFER_SHOT -> waitSubwooferShotRequest();
      default -> {
        if (!getState().climbing
            && getState() != RobotState.IDLE_NO_GP
            && getState() != RobotState.WAITING_PODIUM_SHOT
            && getState() != RobotState.WAITING_SUBWOOFER_SHOT
            && !getState().shootingMode) {
          if (getState().hasNote) {
            setStateFromRequest(RobotState.IDLE_WITH_GP);
          } else {
            setStateFromRequest(RobotState.IDLE_NO_GP);
          }
        }
      }
    }
  }

  public void cancelWaitingFloorShotRequest() {

    switch (getState()) {
      case FLOOR_SHOOT -> {}
      default -> stowRequest();
    }
  }

  public void preloadNoteRequest() {
    if (!getState().climbing) {
      setStateFromRequest(RobotState.IDLE_WITH_GP);
    }
  }

  public void confirmShotRequest() {
    switch (getState()) {
      case WAITING_SUBWOOFER_SHOT -> subwooferShotRequest();
      case WAITING_PODIUM_SHOT -> podiumShotRequest();
      case WAITING_AMP_SHOT -> ampShotRequest();
      case WAITING_FLOOR_SHOT -> floorShotRequest();
      case WAITING_SHOOTER_AMP -> shooterAmpRequest();
      default -> speakerShotRequest();
    }
  }

  public void stopIntakingRequest() {
    switch (getState()) {
      case WAITING_FLOOR_SHOT -> {}
      default -> {
        if (!getState().climbing) {

          switch (getState()) {
            case FINISH_INTAKING, IDLE_WITH_GP -> {}
            default -> {
              setStateFromRequest(RobotState.LAZY_INTAKING);
            }
          }
        }
      }
    }
  }

  public void climb1LineupOutterRequest() {
    setStateFromRequest(RobotState.CLIMB_1_LINEUP_OUTER);
  }

  public void climb2LineupInnerRequest() {
    setStateFromRequest(RobotState.CLIMB_2_LINEUP_INNER);
  }

  public void climb3LineupFinalRequest() {
    setStateFromRequest(RobotState.CLIMB_3_LINEUP_FINAL);
  }

  public void climb4HangingRequest() {
    setStateFromRequest(RobotState.CLIMB_4_HANGING);
  }

  public void climb5HangingTrapScoreRequest() {
    setStateFromRequest(RobotState.CLIMB_5_HANGING_TRAP_SCORE);
  }

  public void climb6HangingElevatorFinishRequest() {
    setStateFromRequest(RobotState.CLIMB_6_HANGING_FINISHED);
  }

  public void getClimberForwardRequest() {
    switch (getState()) {
      case CLIMB_1_LINEUP_OUTER -> climb2LineupInnerRequest();

      case CLIMB_2_LINEUP_INNER -> climb3LineupFinalRequest();

      case CLIMB_3_LINEUP_FINAL -> climb4HangingRequest();

      case PREPARE_CLIMB_4_HANGING, CLIMB_4_HANGING -> climb5HangingTrapScoreRequest();

      case CLIMB_5_HANGING_TRAP_SCORE -> climb6HangingElevatorFinishRequest();

      case CLIMB_6_HANGING_FINISHED -> {}
      default ->
          // Start climb sequence
          climb1LineupOutterRequest();
    }
  }

  public void getClimberBackwardRequest() {
    switch (getState()) {
      case CLIMB_1_LINEUP_OUTER -> stowRequest();

      case CLIMB_2_LINEUP_INNER -> climb1LineupOutterRequest();

      case CLIMB_3_LINEUP_FINAL -> climb2LineupInnerRequest();

      case CLIMB_4_HANGING -> climb3LineupFinalRequest();
      case CLIMB_5_HANGING_TRAP_SCORE -> climb4HangingRequest();
      case CLIMB_6_HANGING_FINISHED -> climb5HangingTrapScoreRequest();
      default -> {}
    }
  }

  public Command waitForStateCommand(RobotState goalState) {
    return Commands.waitUntil(() -> getState() == goalState);
  }

  @Override
  public void teleopInit() {
    if (RobotConfig.IS_DEVELOPMENT && !getState().climbing) {
      // Stow when entering teleop, but only when we're in dev mode and not mid-climb
      // Helps to avoid evil stuff happening after exiting auto
      stowRequest();
    }
  }
}

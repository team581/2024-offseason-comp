// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_manager;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.conveyor.ConveyorState;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.redirect.RedirectState;
import frc.robot.redirect.RedirectSubsystem;
import frc.robot.util.FlagManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class NoteManager extends LifecycleSubsystem {
  private final FlagManager<NoteFlag> flags = new FlagManager<>("NoteManager", NoteFlag.class);
  

  public final QueuerSubsystem queuer;
  public final IntakeSubsystem intake;
  public final ConveyorSubsystem conveyor;
  public final RedirectSubsystem redirect;

  private NoteState state = NoteState.IDLE_NO_GP;

  public NoteManager(
      QueuerSubsystem queuer,
      IntakeSubsystem intake,
      ConveyorSubsystem conveyor,
      RedirectSubsystem redirect) {
    super(SubsystemPriority.ROBOT_MANAGER);
    this.queuer = queuer;
    this.intake = intake;
    this.conveyor = conveyor;
    this.redirect = redirect;

    
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("NoteManager/State", state);
    flags.log();

    
    // State transitions from requests
    for (NoteFlag flag : flags.getChecked()) {
      switch (flag) {
        case AMP_SCORE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.AMP_SCORING;
          }
          break;
        case AMP_WAIT:
        case TRAP_WAIT:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            // Do nothing, you are already idling with the note in the conveyor
          } else {
            state = NoteState.QUEUER_TO_CONVEYOR;
          }
          break;
        case TRAP_SCORE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.TRAP_SCORING;
          }
          break;
        case IDLE_IN_QUEUER:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER;
          } else {
            state = NoteState.IDLE_IN_QUEUER;
          }
          break;
        
        case IDLE_NO_GP:
          state = NoteState.IDLE_NO_GP;
          break;
        case LAZY_INTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER;
          } else {
            state = NoteState.LAZY_INTAKE_TO_QUEUER;
          }
          break;
        case INTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.AMP_SCORING;
          } else if (state == NoteState.INTAKE_TO_QUEUER) {
            // A note is already in the intake and being passed to the queuer, so we should ignore
            // the request
          } else {
            state = NoteState.GROUND_NOTE_TO_INTAKE;
          }
          break;
        case OUTTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER_FOR_OUTTAKING;
          } else {
            state = NoteState.QUEUER_TO_INTAKE_FOR_OUTTAKING;
          }
          break;
        case SHOOTER_OUTTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKING;
          } else {
            state = NoteState.SHOOTER_OUTTAKING;
          }
          break;
        case SHOOTER_SCORE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER_FOR_SHOOTING;
          } else {
            state = NoteState.SHOOTING;
          }
          break;
        case DROP:
          state = NoteState.DROPPING;
          break;
        case UNJAM:
          state = NoteState.UNJAM;
          break;
      }
    }

    DogLog.log("NoteManager/StateAfterFlags", state);

    // Automatic state transitions
    switch (state) {
      case OUTTAKING:
      case IDLE_NO_GP:
      case IDLE_IN_CONVEYOR:
      
      case IDLE_IN_QUEUER:
      case TRAP_SCORING:
      case UNJAM:
        // Do nothing
        break;
      case AMP_SCORING:
        if (!conveyor.hasNote()) {
          state = NoteState.IDLE_NO_GP;
        }
        break;
      case SHOOTING:
      case DROPPING:
      case SHOOTER_OUTTAKING:
        if (!queuer.hasNote() && !intake.hasNote()) {
          state = NoteState.IDLE_NO_GP;
        }
        break;
      case INTAKE_TO_QUEUER:
      case LAZY_INTAKE_TO_QUEUER:
        if (queuer.hasNote()) {
          state = NoteState.IDLE_IN_QUEUER;
        }
        break;
      case GROUND_NOTE_TO_INTAKE:
        if (queuer.hasNote()) {
          // Trying to restart the intake sequence, even though a note is already fully inside the
          // robot
          state = NoteState.IDLE_IN_QUEUER;
        } else if (intake.hasNote()) {
          state = NoteState.INTAKE_TO_QUEUER;
        }
        break;
      case QUEUER_TO_INTAKE_FOR_OUTTAKING:
        if (!queuer.hasNote() && intake.hasNote()) {
          state = NoteState.OUTTAKING;
        }
        break;
      case INTAKE_TO_QUEUER_FOR_SHOOTING:
        if (queuer.hasNote()) {
          state = NoteState.SHOOTING;
        }
        break;
      case CONVEYOR_TO_QUEUER_FOR_OUTTAKING:
        if (queuer.hasNote()) {
          state = NoteState.OUTTAKING;
        }
        break;
      case CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKING:
        if (queuer.hasNote()) {
          state = NoteState.SHOOTER_OUTTAKING;
        }
        break;
      case CONVEYOR_TO_QUEUER_FOR_SHOOTING:
        if (queuer.hasNote()) {
          state = NoteState.SHOOTING;
        }
        break;
      case CONVEYOR_TO_QUEUER:
        if (queuer.hasNote()) {
          state = NoteState.IDLE_IN_QUEUER;
        }
        break;
      case QUEUER_TO_CONVEYOR:
        if (conveyor.hasNote()) {
          state = NoteState.IDLE_IN_CONVEYOR;
        }
        break;
      default:
        break;
    }

    DogLog.log("NoteManager/StateAfterTransitions", state);

    // State actions
    switch (state) {
      case IDLE_NO_GP:
      case IDLE_IN_CONVEYOR:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        queuer.setState(QueuerState.IDLE);
        redirect.setState(RedirectState.IDLE);
        break;
      case IDLE_IN_QUEUER:
        if (queuer.hasNote()) {
          intake.setState(IntakeState.IDLE);
          conveyor.setState(ConveyorState.IDLE);
       
        queuer.setState(QueuerState.INTAKING);
        redirect.setState(RedirectState.IDLE);}
        break;
      
      case LAZY_INTAKE_TO_QUEUER:
        intake.setState(IntakeState.TO_QUEUER_SLOW);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.INTAKING);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case INTAKE_TO_QUEUER:
      case GROUND_NOTE_TO_INTAKE:
        intake.setState(IntakeState.TO_QUEUER);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.INTAKING);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case AMP_SCORING:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.AMP_SHOT);
        queuer.setState(QueuerState.IDLE);
        redirect.setState(RedirectState.TO_CONVEYOR);
        break;
      case TRAP_SCORING:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.TRAP_SHOT_PULSE);
        queuer.setState(QueuerState.IDLE);
        redirect.setState(RedirectState.TO_CONVEYOR);
        break;
      case OUTTAKING:
      case QUEUER_TO_INTAKE_FOR_OUTTAKING:
        intake.setState(IntakeState.OUTTAKING);
        conveyor.setState(ConveyorState.QUEUER_TO_INTAKE);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        redirect.setState(RedirectState.TO_CONVEYOR);
        break;
      case DROPPING:
      case SHOOTER_OUTTAKING:
        intake.setState(IntakeState.TO_QUEUER);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case SHOOTING:
        intake.setState(IntakeState.TO_QUEUER_SHOOTING);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case INTAKE_TO_QUEUER_FOR_SHOOTING:
        intake.setState(IntakeState.TO_QUEUER_SHOOTING);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.INTAKING);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case UNJAM:
        intake.setState(IntakeState.TO_QUEUER);
        conveyor.setState(ConveyorState.AMP_SHOT);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case QUEUER_TO_CONVEYOR:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.INTAKE_TO_SELF);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        redirect.setState(RedirectState.TO_CONVEYOR);
        break;
      case CONVEYOR_TO_QUEUER:
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.INTAKING);
        conveyor.setState(ConveyorState.CONVEYOR_TO_REDIRECT);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case CONVEYOR_TO_QUEUER_FOR_OUTTAKING:
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.INTAKING);
        conveyor.setState(ConveyorState.CONVEYOR_TO_REDIRECT);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKING:
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.INTAKING);
        conveyor.setState(ConveyorState.CONVEYOR_TO_REDIRECT);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      case CONVEYOR_TO_QUEUER_FOR_SHOOTING:
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        conveyor.setState(ConveyorState.CONVEYOR_TO_REDIRECT);
        redirect.setState(RedirectState.TO_QUEUER);
        break;
      default:
        break;
    }

    // Reset all flags
    flags.clear();
  }

  public void intakeRequest() {
    flags.check(NoteFlag.INTAKE);
  }

  public void shooterScoreRequest() {
    flags.check(NoteFlag.SHOOTER_SCORE);
  }

  public void shooterOuttakeRequest() {
    flags.check(NoteFlag.SHOOTER_OUTTAKE);
  }

  
  public void idleInQueuerRequest() {
    flags.check(NoteFlag.IDLE_IN_QUEUER);
  }

  public void idleNoGPRequest() {
    flags.check(NoteFlag.IDLE_NO_GP);
  }

  public void ampScoreRequest() {
    flags.check(NoteFlag.AMP_SCORE);
  }

  public void trapShotRequest() {
    flags.check(NoteFlag.TRAP_SCORE);
  }

  public void trapWaitRequest() {
    flags.check(NoteFlag.TRAP_WAIT);
  }

  public void ampWaitRequest() {
    flags.check(NoteFlag.AMP_WAIT);
  }

  public void outtakeRequest() {
    flags.check(NoteFlag.OUTTAKE);
  }

  public void lazyIntakeRequest() {
    flags.check(NoteFlag.LAZY_INTAKE);
  }

  public void dropRequest() {
    flags.check(NoteFlag.DROP);
  }

  public NoteState getState() {
    return state;
  }

  public void evilStateOverride(NoteState newState) {
    state = newState;
  }

  public void unjamRequest() {
    flags.check(NoteFlag.UNJAM);
  }
}

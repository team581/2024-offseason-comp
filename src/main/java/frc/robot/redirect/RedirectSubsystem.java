// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.redirect;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class RedirectSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private RedirectState goalState = RedirectState.IDLE;

  public RedirectSubsystem(TalonFX motor) {
    super(SubsystemPriority.REDIRECT);

    motor.getConfigurator().apply(RobotConfig.get().redirect().motorConfig());

    this.motor = motor;
  }

  @Override
  public void robotPeriodic() {
    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case QUEUER_TO_CONVEYOR:
      case INTAKE_TO_QUEUER:
      case QUEUER_TO_SHOOTER:
        motor.setVoltage(6);
        break;
      case TO_OUTTAKE:
      case CONVEYOR_TO_QUEUER:
        motor.setVoltage(-6);
      default:
        break;
    }
  }

  public void setState(RedirectState state) {
    goalState = state;
  }

  public RedirectState getState() {
    return goalState;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.redirect;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
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
      case TO_CONVEYOR:
        motor.setVoltage(0);
        break;
      case TO_QUEUER:
        motor.setVoltage(-0);
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

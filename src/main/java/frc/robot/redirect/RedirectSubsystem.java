package frc.robot.redirect;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;





public class RedirectSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;

  public RedirectSubsystem(TalonFX motor){
    super(SubsystemPriority.REDIRECT);

    motor.getConfigurator().apply(RobotConfig.get().redirect().motorConfig())

  }
}

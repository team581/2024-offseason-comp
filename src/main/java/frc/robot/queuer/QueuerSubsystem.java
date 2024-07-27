// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class QueuerSubsystem extends LifecycleSubsystem {
  
  private final TalonFX motor;
  private final DigitalInput sensor;
  private QueuerState goalState = QueuerState.IDLE;
  private final Debouncer debouncer = RobotConfig.get().queuer().debouncer();
  private boolean debouncedSensor = false;
  

  public QueuerSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.QUEUER);

    motor.getConfigurator().apply(RobotConfig.get().queuer().motorConfig());

    this.sensor = sensor;
    this.motor = motor;

  }

  @Override
  public void robotPeriodic() {
    debouncedSensor = debouncer.calculate(sensorHasNote());
    // TODO: We accidentally were calling .calculate() twice for a very long time, and don't have
    // time to validate behavior when we call it just once
    debouncer.calculate(sensorHasNote());
    DogLog.log("Queuer/State", goalState);
    DogLog.log("Queuer/SensorHasNote", sensorHasNote());

    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case INTAKING:
        if (hasNote()) {
          motor.disable();
        } else {
          motor.setVoltage(1);
        }
        break;
      
        
      case PASS_TO_INTAKE:
        motor.setVoltage(-1);
        break;
      case PASS_TO_SHOOTER:
        motor.setVoltage(12);
        break;
      default:
        break;
    }
  }

  public void setState(QueuerState state) {
    goalState = state;
  }

  public boolean hasNote() {
    return debouncedSensor;
  }

  public boolean sensorHasNote() {
    return sensor.get();
  }

  public QueuerState getState() {
    return goalState;
  }
}

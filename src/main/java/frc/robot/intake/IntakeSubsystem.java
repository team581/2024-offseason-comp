// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final CANSparkMax intakeCentering;
  private final DigitalInput sensor;
  private final Debouncer debouncer = RobotConfig.get().intake().debouncer();
  private boolean debouncedSensor = false;
  private IntakeState goalState = IntakeState.IDLE;

  public IntakeSubsystem(
      TalonFX motor,
      CANSparkMax intakeCentering,
      DigitalInput sensor) {
    super(SubsystemPriority.INTAKE);

    motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());
    intakeCentering.setSmartCurrentLimit(20);
    intakeCentering.burnFlash();


    this.motor = motor;
    this.intakeCentering = intakeCentering;

    this.sensor = sensor;
  }

  @Override
  public void robotPeriodic() {
    debouncedSensor = debouncer.calculate(sensorHasNote());
    DogLog.log("Intake/State", goalState);
    DogLog.log("Intake/DebouncedHasNote", debouncedSensor);
    DogLog.log("Intake/HasNote", hasNote());
    DogLog.log("Intake/SensorHasNote", sensorHasNote());
    DogLog.log("Intake/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/Voltage", motor.getMotorVoltage().getValueAsDouble());

    switch (goalState) {
      case IDLE:
        motor.disable();
        intakeCentering.disable();

        break;
      case OUTTAKING:
        motor.setVoltage(-6);
        intakeCentering.setVoltage(-6);

        break;
      case FROM_QUEUER:
        motor.setVoltage(-4); // -3
        intakeCentering.setVoltage(-4);

        break;
      case FROM_CONVEYOR:
        motor.setVoltage(-8);
        intakeCentering.setVoltage(-8);

        break;
      case TO_QUEUER:
        if (hasNote()) {
          motor.setVoltage(10);
          intakeCentering.setVoltage(10);

        } else {
          motor.setVoltage(12);
          intakeCentering.setVoltage(12);

        }
        break;
      case TO_QUEUER_SLOW:
        if (hasNote()) {
          motor.setVoltage(10);
          intakeCentering.setVoltage(10);

        } else {
          motor.setVoltage(5);
          intakeCentering.setVoltage(5);

        }
        break;

      case TO_CONVEYOR:
        motor.setVoltage(3); // 2
        intakeCentering.setVoltage(3);

        break;
      case TO_QUEUER_SHOOTING:
        motor.setVoltage(8);
        intakeCentering.setVoltage(8);

        break;

      default:
        break;
    }
  }

  public void setState(IntakeState state) {
    goalState = state;
  }

  public boolean hasNote() {
    switch (goalState) {
      case TO_QUEUER:
      case TO_QUEUER_SHOOTING:
      case TO_QUEUER_SLOW:
        // Bypass debouncer when we are sending game piece to queuer
        return sensorHasNote();
      default:
        return debouncedSensor;
    }
  }

  public boolean sensorHasNote() {
    return sensor.get();
  }

  public IntakeState getState() {
    return goalState;
  }
}

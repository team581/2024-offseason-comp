// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.snaps;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.fms.FmsSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.function.DoubleSupplier;

public class SnapManager extends LifecycleSubsystem {
  public static double getSourceAngle() {
    return FmsSubsystem.isRedAlliance() ? 60.0 : (180.0 - 60.0);
  }

  public static double getStageLeftAngle() {
    return FmsSubsystem.isRedAlliance() ? -60 : (-60 + 180);
  }

  public static double getStageRightAngle() {
    return FmsSubsystem.isRedAlliance() ? 60.0 : (60.0 + 180.0);
  }

  public static double getAmpAngle() {
    return FmsSubsystem.isRedAlliance() ? -90 : (-90.0);
  }

  public static double getStageBackChain() {
    return FmsSubsystem.isRedAlliance() ? 180 : (0);
  }

  public static double getPodiumAngle() {
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getPresetAmpAngle() {
    return FmsSubsystem.isRedAlliance() ? 315 : (315 + 180.0);
  }

  private final SwerveSubsystem swerve;
  private double angle = 0;
  private boolean enabled;
  private final CommandXboxController controller;

  public SnapManager(SwerveSubsystem swerve, CommandXboxController driverController) {
    super(SubsystemPriority.SNAPS);
    this.swerve = swerve;
    this.controller = driverController;

    new Trigger(() -> Math.abs(controller.getRightX()) > 0.075).onTrue(getDisableCommand());
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }

  public void setEnabled(boolean value) {
    this.enabled = value;

    if (!value) {
      swerve.disableSnapToAngle();
    }
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("SnapManager/Enabled", enabled);
    DogLog.log("SnapManager/GoalAngle", angle);

    if (enabled) {
      swerve.setSnapToAngle(angle);
    }
  }

  public Command getCommand(DoubleSupplier angle) {
    return run(() -> {
          setAngle(angle.getAsDouble());
          setEnabled(true);
        })
        .withName("SnapCommand");
  }

  public Command getDisableCommand() {
    return runOnce(() -> setEnabled(false)).withName("SnapDisableCommand");
  }

  @Override
  public void teleopInit() {
    // Avoid sudden snaps as soon as you enable
    setEnabled(false);
  }

  public void cancelCurrentCommand() {
    Command command = getCurrentCommand();

    if (command != null) {
      command.cancel();
    }
  }
}

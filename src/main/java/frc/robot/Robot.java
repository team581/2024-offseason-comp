// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Autos;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_manager.NoteManager;
import frc.robot.note_map_manager.NoteMapManager;
import frc.robot.note_tracking.NoteTrackingManager;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.redirect.RedirectSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.Stopwatch;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.wrist.WristSubsystem;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final RumbleControllerSubsystem driverRumble =
      new RumbleControllerSubsystem(driverController, false);
  private final RumbleControllerSubsystem operatorRumble =
      new RumbleControllerSubsystem(operatorController, true);

  private final WristSubsystem wrist =
      new WristSubsystem(
          new TalonFX(RobotConfig.get().wrist().motorID(), RobotConfig.get().canivoreName()));
  private final ShooterSubsystem shooter =
      new ShooterSubsystem(
          new TalonFX(RobotConfig.get().shooter().leftMotorID(), RobotConfig.get().canivoreName()),
          new TalonFX(
              RobotConfig.get().shooter().rightMotorID(), RobotConfig.get().canivoreName()));

  private final ClimberSubsystem climber =
      new ClimberSubsystem(
          new TalonFX(RobotConfig.get().climber().leftMotorID(), RobotConfig.get().canivoreName()),
          new TalonFX(
              RobotConfig.get().climber().rightMotorID(), RobotConfig.get().canivoreName()));
  private final RedirectSubsystem redirect =
      new RedirectSubsystem(new TalonFX(RobotConfig.get().redirect().motorID(), "rio"));
  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          new TalonFX(RobotConfig.get().intake().mainMotorID(), RobotConfig.get().canivoreName()),
          new CANSparkMax(RobotConfig.get().intake().centeringMotorID(), MotorType.kBrushed),
          new DigitalInput(RobotConfig.get().intake().sensorID()));
  private final SwerveSubsystem swerve = new SwerveSubsystem(driverController);
  private final ImuSubsystem imu = new ImuSubsystem(swerve);
  private final FmsSubsystem fms = new FmsSubsystem();
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          new TalonFX(RobotConfig.get().elevator().motorID(), RobotConfig.get().canivoreName()));
  private final QueuerSubsystem queuer =
      new QueuerSubsystem(
          new TalonFX(RobotConfig.get().queuer().motorID(), RobotConfig.get().canivoreName()),
          new DigitalInput(RobotConfig.get().queuer().sensorID()));
  private final ConveyorSubsystem conveyor =
      new ConveyorSubsystem(
          new TalonFX(RobotConfig.get().conveyor().motorID(), "rio"),
          new DigitalInput(RobotConfig.get().conveyor().sensorID()));
  private final VisionSubsystem vision = new VisionSubsystem(imu);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(swerve, imu, vision);
  private final SnapManager snaps = new SnapManager(swerve, driverController);
  private final NoteManager noteManager = new NoteManager(queuer, intake, conveyor, redirect);
  private final RobotManager robotManager =
      new RobotManager(
          wrist, elevator, shooter, localization, vision, climber, swerve, snaps, imu, noteManager);
  private final RobotCommands actions = new RobotCommands(robotManager);
  private final NoteTrackingManager noteTrackingManager =
      new NoteTrackingManager(localization, swerve);
  private final NoteMapManager noteMapManager =
      new NoteMapManager(actions, noteTrackingManager, robotManager, localization, snaps);
  private final Autos autos =
      new Autos(swerve, localization, actions, robotManager, noteMapManager, noteTrackingManager);
  private final LightsSubsystem lightsSubsystem =
      new LightsSubsystem(
          new CANdle(RobotConfig.get().lights().deviceID(), "rio"), robotManager, vision, intake);

  public Robot() {
    System.out.println("roboRIO serial number: " + RobotConfig.SERIAL_NUMBER);

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureNt(false)
            .withCaptureDs(true)
            .withNtPublish(RobotConfig.IS_DEVELOPMENT));
    DogLog.setPdh(new PowerDistribution());

    // Record metadata
    DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("Metadata/RoborioSerialNumber", RobotConfig.SERIAL_NUMBER);
    DogLog.log("Metadata/RobotName", RobotConfig.get().robotName());
    DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        DogLog.log("Metadata/GitDirty", "All changes committed");
        break;
      case 1:
        DogLog.log("Metadata/GitDirty", "Uncomitted changes");
        break;
      default:
        DogLog.log("Metadata/GitDirty", "Unknown");
        break;
    }

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();
  }

  @Override
  public void robotInit() {
    noteMapManager.off();
    // var now = Timer.getFPGATimestamp();
    // DogLog.timestamp("NoteMapManager/RobotInit");
    // noteTrackingManager.resetNoteMap(
    //     new ArrayList<>(
    //         List.of(
    //             new NoteMapElement(now + 8, AutoNoteStaged.noteIdToTranslation(4)),
    //             new NoteMapElement(now + 5, AutoNoteStaged.noteIdToTranslation(5)),
    //             new NoteMapElement(now + 3, AutoNoteStaged.noteIdToTranslation(6))

    //           )));
    // var steps = new LinkedList<AutoNoteStep>();
    // steps.add(AutoNoteStep.score(7, 6));
    // steps.add(AutoNoteStep.score(4));
    // noteMapManager.setSteps(steps);
  }

  @Override
  public void robotPeriodic() {
    Stopwatch.getInstance().start("Scheduler/CommandSchedulerPeriodic");
    CommandScheduler.getInstance().run();
    Stopwatch.getInstance().stop("Scheduler/CommandSchedulerPeriodic");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = autos.getAutoCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    noteMapManager.off();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.driveTeleopCommand());

    driverController.back().onTrue(localization.getZeroCommand());

    driverController.y().onTrue(snaps.getCommand(() -> SnapManager.getSourceAngle()));
    driverController.x().onTrue(snaps.getCommand(() -> SnapManager.getStageLeftAngle()));
    driverController.b().onTrue(snaps.getCommand(() -> SnapManager.getStageRightAngle()));
    driverController.a().onTrue(snaps.getCommand(() -> SnapManager.getAmpAngle()));
    driverController.povUp().onTrue(snaps.getCommand(() -> SnapManager.getStageBackChain()));

    driverController
        .leftTrigger()
        .onTrue(
            actions
                .intakeCommand()
                .alongWith(
                    robotManager
                        .waitForStateCommand(RobotState.INTAKING)
                        .andThen(Commands.waitUntil(() -> robotManager.getState().hasNote))
                        .andThen(driverRumble.getRumbleShortCommand())))
        .onFalse(
            actions
                .stopIntakingCommand()
                .alongWith(Commands.waitUntil(() -> robotManager.getState().hasNote))
                .andThen(driverRumble.getRumbleShortCommand()));
    driverController
        .rightTrigger()
        .onTrue(actions.confirmShotCommand())
        .onFalse(actions.stopShootingCommand());
    driverController
        .rightBumper()
        .onTrue(actions.shooterOuttakeCommand())
        .onFalse(actions.stowCommand());

    operatorController.povUp().onTrue(actions.getClimberForwardCommand());
    operatorController.povDown().onTrue(actions.getClimberBackwardCommand());

    operatorController
        .leftTrigger()
        .whileTrue(noteMapManager.testCommand())
        .onFalse(
            actions
                .stowCommand()
                .alongWith(
                    Commands.runOnce(
                        () -> {
                          noteMapManager.off();
                        })));
    operatorController.a().onTrue(actions.stowCommand());
    operatorController.b().onTrue(actions.unjamCommand()).onFalse(actions.idleNoGPCommand());
    operatorController.povLeft().onTrue(actions.unjamCommand()).onFalse(actions.idleNoGPCommand());
    operatorController
        .povRight()
        .onTrue(actions.shooterStoppedUnjamCommand())
        .onFalse(actions.idleNoGPCommand());
    operatorController
        .y()
        .onTrue(actions.waitSubwooferShotCommand())
        .onFalse(actions.stowCommand());
    operatorController
        .rightTrigger()
        .onTrue(actions.waitForSpeakerShotCommand())
        .onFalse(actions.stowCommand());
    operatorController.rightBumper().onTrue(actions.waitForAmpShotCommand());
    operatorController.x().onTrue(actions.outtakeCommand()).onFalse(actions.stowCommand());
    operatorController
        .leftBumper()
        .onTrue(actions.waitForFloorShotCommand())
        .onFalse(actions.cancelWaitingFloorShotCommand());
    operatorController.back().onTrue(actions.homeCommand());
  }
}

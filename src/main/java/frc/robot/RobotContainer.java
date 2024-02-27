// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.test.DriveSubsystemTest;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.auto.MiddleC1C2;
import frc.robot.commands.auto.MiddleTwoNote;
import frc.robot.commands.auto.Sub1TwoNote;
import frc.robot.commands.auto.Sub3TwoNote;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import java.util.Map;
import java.util.Map.Entry;

/**
 * Constructs a new RobotContainer object. This constructor is responsible for initializing
 * controllers, subsystems, shuffleboard tabs, and configuring bindings.
 */
public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  private final PowerDistributionSubsystem powerDistributionSubsystem =
      new PowerDistributionSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
  private final ObjectDetectionSubsystem objectDetectionSubsystem =
      new ObjectDetectionSubsystem(VisionConstants.OBJECT_DETECTION_CAMERA);
  private final PoseEstimatorSubsystem poseEstimatorSubsystem =
      new PoseEstimatorSubsystem(driveSubsystem, FieldPositions.getInstance());
  private final BlinkinSubsystem lightsSubsystem = new BlinkinSubsystem();

  private boolean isTeleop;

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  private Map<String, Command> namedCommands =
      Map.of(
          "intakeAndShootPodium",
              new SequentialCommandGroup(
                  manipulatorSubsystem.getIntakeCommand(),
                  manipulatorSubsystem.getPodiumShootCommand()),
          "intake", manipulatorSubsystem.getIntakeCommand(),
          "shootWingNote", manipulatorSubsystem.getPodiumShootCommand(),
          "shootStage", manipulatorSubsystem.getStageShootCommand(),
          "autoShoot",
              new AutoShootCommand(driveSubsystem, manipulatorSubsystem, poseEstimatorSubsystem));

  public RobotContainer() {
    setupShuffleboardTab(RobotConstants.COMPETITION_TAB);
    configurePathPlanner();
    buildAutoList();
    buildTestList();
    configureDriverBindings();
    configureOperatorBindings();
  }

  /**
   * Sets up the Shuffleboard tab with the specified commands.
   *
   * @param tab The ShuffleboardTab to add commands to.
   */
  private void setupShuffleboardTab(ShuffleboardTab tab) {
    tab.add("Home Climber", manipulatorSubsystem.getClimberHomeCommand());
  }

  /**
   * Configures the path planner for autonomous routines. This method sets up holonomic path
   * following with specified PID constants, replanning configurations, etc.
   */
  private void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        driveSubsystem::getPose,
        driveSubsystem::resetOdometry,
        driveSubsystem::getRobotRelativeSpeeds,
        driveSubsystem::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.0, 0.0, 0.0),
            new PIDConstants(1.0, 0.0, 0.0),
            2.5,
            0.4,
            new ReplanningConfig()),
        () -> AllianceColor.alliance == Alliance.Red,
        driveSubsystem);

    for (Entry<String, Command> entry : namedCommands.entrySet()) {
      NamedCommands.registerCommand(entry.getKey(), new ScheduleCommand(entry.getValue()));
    }
  }

  /** Builds the list of autonomous command options for the SendableChooser. */
  private void buildAutoList() {
    autoCommandChooser.addOption("0 - NoOp", new InstantCommand());
    autoCommandChooser.addOption("1 - MiddleC1C2", MiddleC1C2.get(manipulatorSubsystem));
    autoCommandChooser.addOption("2 - MidleTwoNote", MiddleTwoNote.get(manipulatorSubsystem));
    autoCommandChooser.addOption("3 - Sub1TwoNote", Sub1TwoNote.get(manipulatorSubsystem));
    autoCommandChooser.addOption("4 - Sub3TwoNote", Sub3TwoNote.get(manipulatorSubsystem));

    RobotConstants.COMPETITION_TAB
        .add("Auto Command", autoCommandChooser)
        .withSize(4, 1)
        .withPosition(0, 1);
  }

  /** Builds the list of test commands for the Test tab. */
  private void buildTestList() {
    RobotConstants.TEST_TAB
        .add(
            "Drive Subsystem Test", new DriveSubsystemTest(driveSubsystem, RobotConstants.TEST_TAB))
        .withPosition(2, 1)
        .withSize(2, 1);
  }

  /**
   * Configures button bindings for the driver controller, setting up commands for different button
   * combinations.
   */
  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(getDefaultDriveSubsystemCommand());

    driverController
        .leftStick()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.NORMAL)));

    driverController
        .rightStick()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURTLE)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.NORMAL)));

    driverController
        .leftBumper()
        .whileTrue(manipulatorSubsystem.getSubwooferShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .povUp()
        .whileTrue(manipulatorSubsystem.getPodiumShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .leftTrigger()
        .whileTrue(manipulatorSubsystem.getIntakeCommand())
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    driverController
        .rightTrigger()
        .whileTrue(
            new AutoAimCommand(
                driveSubsystem,
                poseEstimatorSubsystem,
                () -> -MathUtil.applyDeadband(powWithSign(driverController.getLeftX(), 2), 0.01),
                () -> -MathUtil.applyDeadband(powWithSign(driverController.getLeftY(), 2), 0.01)));

    driverController.a().onTrue(new InstantCommand(() -> driveSubsystem.resetImu()));

    driverController
        .povLeft()
        .whileTrue(manipulatorSubsystem.getStageShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .povDown()
        .whileTrue(manipulatorSubsystem.getRollerRunCommand(IntakeSpeed.REVERSE))
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    driverController
        .rightBumper()
        .whileTrue(
            new AutoShootCommand(
                driveSubsystem,
                manipulatorSubsystem,
                poseEstimatorSubsystem,
                () -> -MathUtil.applyDeadband(powWithSign(driverController.getLeftX(), 2), 0.01),
                () -> -MathUtil.applyDeadband(powWithSign(driverController.getLeftY(), 2), 0.01)));

    new Trigger(() -> manipulatorSubsystem.isNoteInFeeder())
        .onTrue(new InstantCommand(() -> lightsSubsystem.signalNoteInHolder()))
        .onFalse(new InstantCommand(() -> lightsSubsystem.reset()));

    new Trigger(() -> manipulatorSubsystem.isNoteInRoller() && isTeleop)
        .onTrue(
            new InstantCommand(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))
        .onFalse(
            new InstantCommand(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  /**
   * Returns the default RunCommand for driving the robot based on controller input.
   *
   * @return The default RunCommand for driving the robot.
   */
  private RunCommand getDefaultDriveSubsystemCommand() {
    return new RunCommand(
        () ->
            driveSubsystem.drive(
                -MathUtil.applyDeadband(powWithSign(driverController.getLeftY(), 2), 0.01),
                -MathUtil.applyDeadband(powWithSign(driverController.getLeftX(), 2), 0.01),
                -MathUtil.applyDeadband(powWithSign(driverController.getRightX(), 2), 0.01)),
        driveSubsystem);
  }

  /**
   * Raise the first argument to the power of the second argument, keeping the sign of the first.
   *
   * @param x A double.
   * @param b A double.
   * @return The result of x^b with the sign of x.
   */
  private double powWithSign(double x, double b) {
    return Math.copySign(Math.pow(x, b), x);
  }

  /**
   * Configures button bindings for the operator controller, setting up commands for different
   * button combinations.
   */
  private void configureOperatorBindings() {
    manipulatorSubsystem.setDefaultCommand(
        manipulatorSubsystem.getClimberRunCommand(
            () -> -MathUtil.applyDeadband(operatorController.getRightY(), 0.01)));

    operatorController
        .x()
        .whileTrue(manipulatorSubsystem.getPodiumShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    operatorController
        .b()
        .whileTrue(manipulatorSubsystem.getSubwooferShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    operatorController
        .a()
        .onTrue(new InstantCommand(() -> lightsSubsystem.signalSource()))
        .onFalse(new InstantCommand((() -> lightsSubsystem.reset())));
  }

  /**
   * Sets the robot to teleop mode and optionally resets the odometry if `mode` is true.
   *
   * @param mode If true, sets the robot to teleop mode and resets odometry.
   */
  public void setTeleop(boolean mode) {
    isTeleop = mode;

    lightsSubsystem.reset();
  }

  /**
   * Gets the selected autonomous command from the SendableChooser.
   *
   * @return The selected autonomous command.
   */
  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }

  /** Initializes the robot when transitioning to the disabled state. Resets controller rumble. */
  public void disabledInit() {
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
}

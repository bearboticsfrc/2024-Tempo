// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.test.DriveSubsystemTest;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.auto.MiddleC1;
import frc.robot.commands.auto.MiddleC1C2;
import frc.robot.commands.auto.MiddleTwoNote;
import frc.robot.commands.auto.Sub1TwoNote;
import frc.robot.commands.auto.Sub3TwoNote;
import frc.robot.commands.auto.Sub3W3W2W1;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * Constructs a new RobotContainer object. This constructor is responsible for initializing
 * controllers, subsystems, shuffleboard tabs, and configuring bindings.
 */
public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  @SuppressWarnings("unused")
  private final PowerDistributionSubsystem powerDistributionSubsystem =
      new PowerDistributionSubsystem();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  @SuppressWarnings("unused")
  private final ObjectDetectionSubsystem objectDetectionSubsystem =
      new ObjectDetectionSubsystem(VisionConstants.OBJECT_DETECTION_CAMERA);

  private final PoseEstimatorSubsystem poseEstimatorSubsystem =
      new PoseEstimatorSubsystem(driveSubsystem, FieldPositions.getInstance());

  private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem();

  private boolean isTeleop;
  private boolean isAutoPathTargeting = false;

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  private Map<String, Command> namedCommands =
      Map.of(
          "intakeAndShootPodium",
          new ScheduleCommand(
              Commands.sequence(
                  manipulatorSubsystem.getIntakeCommand(),
                  manipulatorSubsystem.getPodiumShootCommand())),
          "intake",
          new ScheduleCommand(manipulatorSubsystem.getIntakeCommand()),
          "shootWingNote",
          new ScheduleCommand(manipulatorSubsystem.getPodiumShootCommand()),
          "shootStage",
          new ScheduleCommand(manipulatorSubsystem.getStageShootCommand()),
          "startAim",
          Commands.runOnce(() -> this.setAutoPathTargeting(true)),
          "stopAim",
          Commands.runOnce(() -> this.setAutoPathTargeting(false)),
          "subwooferShoot",
          new ScheduleCommand(manipulatorSubsystem.getSubwooferShootCommand()),
          "autoShoot",
          new ScheduleCommand(new AutoShootCommand(driveSubsystem, manipulatorSubsystem)));

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
    tab.addDouble(
        "Distance to Speaker",
        () ->
            LocationHelper.getDistanceToPose(
                driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter()));
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
            new PIDConstants(5),
            new PIDConstants(5),
            3,
            RobotConstants.SWERVE_RADIUS,
            new ReplanningConfig()),
        AllianceColor::isRedAlliance,
        driveSubsystem);

    NamedCommands.registerCommands(namedCommands);

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    return isAutoPathTargeting
        ? Optional.of(
            LocationHelper.getRotationToTranslation(
                    driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerTranslation())
                .minus(Rotation2d.fromDegrees(180)))
        : Optional.empty();
  }

  /** Builds the list of autonomous command options for the SendableChooser. */
  private void buildAutoList() {
    autoCommandChooser.addOption("0 - NoOp", new InstantCommand());
    autoCommandChooser.addOption("1 - MiddleC1C2", MiddleC1C2.get(manipulatorSubsystem));
    autoCommandChooser.addOption(
        "1.5 - MiddleC1", MiddleC1.get(driveSubsystem, manipulatorSubsystem, true));
    autoCommandChooser.addOption("2 - MidleTwoNote", MiddleTwoNote.get(manipulatorSubsystem));
    autoCommandChooser.addOption(
        "3 - Sub1TwoNote", Sub1TwoNote.get(driveSubsystem, manipulatorSubsystem));
    autoCommandChooser.addOption("4 - Sub3TwoNote", Sub3TwoNote.get(manipulatorSubsystem));
    autoCommandChooser.addOption(
        "5 - Sub3W3W2W1",
        Sub3W3W2W1.get(driveSubsystem, manipulatorSubsystem, poseEstimatorSubsystem, false));
    autoCommandChooser.addOption(
        "6 - Sub3W3W2W1C1",
        Sub3W3W2W1.get(driveSubsystem, manipulatorSubsystem, poseEstimatorSubsystem, true));

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
        .whileTrue(
            Commands.runOnce(() -> driveSubsystem.setSpeedMode(SpeedMode.NORMAL), driveSubsystem))
        .onFalse(
            Commands.runOnce(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO), driveSubsystem));

    driverController
        .rightStick()
        .whileTrue(
            Commands.runOnce(() -> driveSubsystem.setSpeedMode(SpeedMode.TURTLE), driveSubsystem))
        .onFalse(
            Commands.runOnce(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO), driveSubsystem));

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
                    FieldPositions.getInstance().getSpeakerTranslation(),
                    () -> getJoystickInput(driverController, JoystickAxis.Ly),
                    () -> getJoystickInput(driverController, JoystickAxis.Lx))
                .repeatedly());

    driverController.a().onTrue(Commands.runOnce(() -> driveSubsystem.resetImu()));
    driverController
        .y()
        .onTrue(manipulatorSubsystem.getPodiumShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .povLeft()
        .whileTrue(manipulatorSubsystem.getStageShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .povDown()
        .whileTrue(manipulatorSubsystem.getRollerRunCommand(IntakeSpeed.REVERSE))
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    driverController
        .povRight()
        .whileTrue(manipulatorSubsystem.getAmpShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .rightBumper()
        .whileTrue(
            Commands.sequence(
                Commands.defer(
                    () ->
                        manipulatorSubsystem.getShooterPrepareCommad(
                            () ->
                                LocationHelper.getDistanceToPose(
                                    driveSubsystem.getPose(),
                                    FieldPositions.getInstance().getSpeakerCenter())),
                    Set.of(manipulatorSubsystem)),
                manipulatorSubsystem.getShootCommand()));

    new Trigger(() -> manipulatorSubsystem.isNoteInFeeder())
        .onTrue(Commands.runOnce(() -> blinkinSubsystem.signalNoteInHolder()))
        .onFalse(Commands.runOnce(() -> blinkinSubsystem.reset()));

    new Trigger(() -> manipulatorSubsystem.isNoteInRoller() && isTeleop)
        .onTrue(
            Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))
        .onFalse(
            Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  /**
   * Returns the default Command for driving the robot based on controller input.
   *
   * @return The default RunCommand for driving the robot.
   */
  private Command getDefaultDriveSubsystemCommand() {
    return Commands.run(
        () ->
            driveSubsystem.drive(
                getJoystickInput(driverController, JoystickAxis.Ly),
                getJoystickInput(driverController, JoystickAxis.Lx),
                getJoystickInput(driverController, JoystickAxis.Rx)),
        driveSubsystem);
  }

  private double getJoystickInput(CommandXboxController controller, JoystickAxis axis) {
    double rawInput;

    switch (axis) {
      case Ly:
        rawInput = driverController.getLeftY();
        break;
      case Lx:
        rawInput = driverController.getLeftX();
        break;
      case Ry:
        rawInput = driverController.getRightY();
        break;
      case Rx:
        rawInput = driverController.getRightX();
        break;
      default:
        rawInput = 0;
    }

    double flippedInput = AllianceColor.isRedAlliance() && axis.isFlipped() ? -rawInput : rawInput;
    return -MathUtil.applyDeadband(powWithSign(flippedInput, 2), 0.01);
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
        .a()
        .onTrue(manipulatorSubsystem.getAmpShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    operatorController
        .y()
        .onTrue(Commands.runOnce(() -> blinkinSubsystem.signalSource()))
        .onFalse(Commands.runOnce(() -> blinkinSubsystem.reset()));

    operatorController.povUp().onTrue(manipulatorSubsystem.getArmRunCommand(ArmPosition.AMP_SHOOT));

    operatorController.povDown().onTrue(manipulatorSubsystem.getArmRunCommand(ArmPosition.HOME));
  }

  /**
   * Sets the robot to teleop mode and optionally resets the odometry if `mode` is true.
   *
   * @param mode If true, sets the robot to teleop mode and resets odometry.
   */
  public void setTeleop(boolean mode) {
    isTeleop = mode;

    blinkinSubsystem.reset();
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

  public void setAutoPathTargeting(boolean isAutoPathTargeting) {
    this.isAutoPathTargeting = isAutoPathTargeting;
  }

  private enum JoystickAxis {
    Ly,
    Lx,
    Ry,
    Rx;

    public boolean isFlipped() {
      return this == Ly || this == Lx;
    }
  }
}

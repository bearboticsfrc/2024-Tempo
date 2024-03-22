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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.bearbotics.fms.AllianceColor;
import frc.bearbotics.test.DriveSubsystemTest;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoNotePickupCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.auto.MiddleC1;
import frc.robot.commands.auto.MiddleTwoNote;
import frc.robot.commands.auto.SmartSub3ToC5C3;
import frc.robot.commands.auto.Sub1C1C2;
import frc.robot.commands.auto.Sub1TwoNote;
import frc.robot.commands.auto.Sub1W1W2C1;
import frc.robot.commands.auto.Sub2W2C3C2;
import frc.robot.commands.auto.Sub2W3W2W1C1;
import frc.robot.commands.auto.Sub3ToC5C3;
import frc.robot.commands.auto.Sub3TwoNote;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.location.FieldPositions;
import frc.robot.location.LocationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.candle.CandlePattern;
import frc.robot.subsystems.candle.CandleSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.IntakeSubsystem.IntakeSpeed;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorSubsystem;
import java.util.Map;
import java.util.Optional;

/**
 * The RobotContainer class serves as the central hub for the robot's system configurations and
 * operations. It initializes all robot subsystems, configures command bindings for both the driver
 * and operator controllers, and sets up autonomous command choices. This class also handles the
 * integration with the Shuffleboard for real-time data display and control adjustments.
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

  private final ObjectDetectionSubsystem objectDetectionSubsystem =
      new ObjectDetectionSubsystem(VisionConstants.OBJECT_DETECTION_CAMERA);

  @SuppressWarnings("unused")
  private final PoseEstimatorSubsystem poseEstimatorSubsystem =
      new PoseEstimatorSubsystem(driveSubsystem, FieldPositions.getInstance());

  private final CandleSubsystem candleSubsystem = new CandleSubsystem();

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
   * Initializes a new RobotContainer. This constructor initializes all subsystems, configures
   * command bindings, and prepares autonomous selections and Shuffleboard tabs.
   */
  private void setupShuffleboardTab(ShuffleboardTab tab) {
    tab.addBoolean("isRedAlliance", () -> AllianceColor.isRedAlliance());
    tab.add("Home Climber", manipulatorSubsystem.getClimberHomeCommand());
    tab.addDouble(
        "Distance to Speaker",
        () ->
            LocationHelper.getDistanceToPose(
                driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerCenter()));
  }

  /**
   * Configures the Shuffleboard tab for competition use. This includes displaying subsystem
   * commands and providing real-time feedback from sensors or subsystem states.
   *
   * @param tab The ShuffleboardTab to add commands and data to.
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
            DriveConstants.MAX_VELOCITY,
            RobotConstants.SWERVE_RADIUS,
            new ReplanningConfig()),
        AllianceColor::isRedAlliance,
        driveSubsystem);

    NamedCommands.registerCommands(namedCommands);

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  /**
   * Sets up the path planner for autonomous operation. This includes configuring the holonomic path
   * following with appropriate PID constants, setting up replanning configurations, and
   * establishing any global overrides.
   */
  public Optional<Rotation2d> getRotationTargetOverride() {
    return isAutoPathTargeting
        ? Optional.of(
            LocationHelper.getRotationToTranslation(
                    driveSubsystem.getPose(), FieldPositions.getInstance().getSpeakerTranslation())
                .minus(Rotation2d.fromDegrees(180)))
        : Optional.empty();
  }

  /**
   * Provides an override for the robot's rotation target during autonomous path following. This can
   * be used to dynamically adjust the robot's orientation based on strategic needs.
   *
   * @return An Optional containing the new rotation target, or an empty Optional if no override is
   *     needed.
   */
  private void buildAutoList() {
    autoCommandChooser.addOption("0 - NoOp", new InstantCommand());
    autoCommandChooser.addOption(
        "1.5 - MiddleC1", MiddleC1.get(driveSubsystem, manipulatorSubsystem, true));
    autoCommandChooser.addOption(
        "2 - " + MiddleTwoNote.NAME, MiddleTwoNote.get(manipulatorSubsystem));
    autoCommandChooser.addOption(
        "3 - Sub1TwoNote", Sub1TwoNote.get(driveSubsystem, manipulatorSubsystem));
    autoCommandChooser.addOption("4 - Sub3TwoNote", Sub3TwoNote.get(manipulatorSubsystem));
    autoCommandChooser.addOption(
        "6 - " + Sub2W3W2W1C1.NAME,
        Sub2W3W2W1C1.get(driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem));
    autoCommandChooser.addOption(
        "7 - " + Sub3ToC5C3.NAME,
        Sub3ToC5C3.get(driveSubsystem, manipulatorSubsystem, objectDetectionSubsystem));
    autoCommandChooser.addOption(
        "7.5 - " + SmartSub3ToC5C3.NAME,
        SmartSub3ToC5C3.get(driveSubsystem, manipulatorSubsystem, objectDetectionSubsystem));
    autoCommandChooser.addOption(
        "8 - " + Sub2W2C3C2.NAME,
        Sub2W2C3C2.get(driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem));
    autoCommandChooser.addOption(
        "9 - " + Sub1C1C2.NAME,
        Sub1C1C2.get(driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem));

    autoCommandChooser.addOption(
        "10 - " + Sub1W1W2C1.NAME,
        Sub1W1W2C1.get(driveSubsystem, objectDetectionSubsystem, manipulatorSubsystem));

    RobotConstants.COMPETITION_TAB
        .add("Auto Command", autoCommandChooser)
        .withSize(4, 1)
        .withPosition(0, 1);
  }

  /**
   * Builds and configures the list of autonomous commands available for selection. This method
   * populates the SendableChooser with pre-defined autonomous routines.
   */
  private void buildTestList() {
    RobotConstants.TEST_TAB
        .add(
            "Drive Subsystem Test", new DriveSubsystemTest(driveSubsystem, RobotConstants.TEST_TAB))
        .withPosition(2, 1)
        .withSize(2, 1);
  }

  /**
   * Sets up a list of test commands for debugging and calibration purposes. These commands are
   * accessible from the Test tab on the Shuffleboard.
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
        .leftBumper()
        .whileTrue(manipulatorSubsystem.getSubwooferShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .povUp()
        .whileTrue(manipulatorSubsystem.getPodiumShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .leftTrigger()
        .whileTrue(manipulatorSubsystem.getManualIntakeCommand())
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    driverController
        .rightTrigger()
        .whileTrue(
            new AutoAimCommand(
                    driveSubsystem,
                    () -> FieldPositions.getInstance().getSpeakerTranslation(),
                    () -> getJoystickInput(driverController, JoystickAxis.Ly),
                    () -> getJoystickInput(driverController, JoystickAxis.Lx))
                .repeatedly());

    driverController.a().onTrue(Commands.runOnce(() -> driveSubsystem.resetImu()));

    driverController
        .b()
        .onTrue(manipulatorSubsystem.getAmpShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    driverController
        .x()
        .whileTrue(
            new AutoAimCommand(
                    driveSubsystem,
                    () -> getJoystickInput(driverController, JoystickAxis.Ly),
                    () -> getJoystickInput(driverController, JoystickAxis.Lx),
                    Rotation2d.fromDegrees(90))
                .repeatedly());

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
            manipulatorSubsystem
                .getShooterAndArmPrepareCommand(
                    () ->
                        LocationHelper.getDistanceToPose(
                            driveSubsystem.getPose(),
                            FieldPositions.getInstance().getSpeakerCenter()))
                .andThen(manipulatorSubsystem.getShootCommand()));

    new Trigger(() -> manipulatorSubsystem.isNoteInFeeder())
        .onTrue(Commands.runOnce(() -> candleSubsystem.setColor(Color.kGreen)))
        .onFalse(Commands.runOnce(() -> candleSubsystem.setAllianceColor()));

    new Trigger(() -> manipulatorSubsystem.isNoteInRoller() && isTeleop)
        .onTrue(
            Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))
        .onFalse(
            Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  /**
   * Configures the button bindings for the driver's Xbox controller. This method maps controller
   * inputs to robot commands for driving, manipulation, and other teleoperated actions.
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

  /**
   * Retrieves joystick input from a specified axis, applies deadband and scaling, and optionally
   * flips the direction based on alliance color. This method helps with processing raw joystick
   * inputs for driving commands.
   *
   * @param controller The CommandXboxController from which to read the input.
   * @param axis The axis (e.g., Ly, Lx) to read from the controller.
   * @return Processed input value from the specified joystick axis.
   */
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
   * Raises a value to a power while preserving the sign, useful for non-linear joystick response
   * curves.
   *
   * @param x The base value.
   * @param b The exponent.
   * @return The result of raising `x` to the power `b`, maintaining the original sign of `x`.
   */
  private double powWithSign(double x, double b) {
    return Math.copySign(Math.pow(x, b), x);
  }

  /**
   * Configures button bindings for the operator's Xbox controller. Similar to driver bindings, this
   * method maps operator inputs to commands for robot manipulation and other functions.
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
        .b()
        .onTrue(manipulatorSubsystem.getBloopShootCommand())
        .onFalse(manipulatorSubsystem.getShootStopCommand());

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(() -> candleSubsystem.setPattern(CandlePattern.STROBE, Color.kGold)))
        .onFalse(Commands.runOnce(() -> candleSubsystem.setAllianceColor()));

    operatorController
        .x()
        .whileTrue(
            Commands.parallel(
                    new AutoNotePickupCommand(
                        driveSubsystem,
                        objectDetectionSubsystem,
                        manipulatorSubsystem::isNoteInRoller),
                    manipulatorSubsystem.getRollerRunCommand(IntakeSpeed.FULL))
                .andThen(manipulatorSubsystem.getIntakeCommand()))
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    operatorController.povUp().onTrue(manipulatorSubsystem.getArmRunCommand(ArmPosition.AMP_SHOOT));

    operatorController.povDown().onTrue(manipulatorSubsystem.getShootStopCommand());

    operatorController
        .leftTrigger()
        .onTrue(manipulatorSubsystem.getFeederRunCommand(IntakeSpeed.REVERSE))
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());

    operatorController
        .rightTrigger()
        .whileTrue(manipulatorSubsystem.getIntakeCommand())
        .onFalse(manipulatorSubsystem.getIntakeStopCommand());
  }

  /**
   * Sets the robot's operational mode to teleoperated and optionally resets odometry.
   *
   * @param mode If true, the robot is set to teleop mode and odometry is reset.
   */
  public void setTeleop(boolean mode) {
    isTeleop = mode;
  }

  /**
   * Retrieves the autonomous command selected from the SendableChooser on the Shuffleboard.
   *
   * @return The Command selected to run in autonomous mode.
   */
  public Command getAutonomousCommand() {
    return autoCommandChooser.getSelected();
  }

  /**
   * Performs initialization tasks when the robot is first started. This includes setting initial
   * subsystem states and configuring global settings.
   */
  public void robotInit() {
    if (manipulatorSubsystem.isNoteInFeeder()) {
      candleSubsystem.setColor(Color.kGreen);
    }
  }

  public void teleopInit() {
    manipulatorSubsystem
        .getShootStopCommand()
        .alongWith(manipulatorSubsystem.getIntakeStopCommand())
        .schedule();
  }

  /** Prepares the robot for being disabled, including stopping any rumble on the controllers. */
  public void disabledInit() {
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
  /**
   * Enables or disables the auto path targeting mode, which affects autonomous path following
   * behavior.
   *
   * @param isAutoPathTargeting If true, enables auto path targeting mode.
   */
  public void setAutoPathTargeting(boolean isAutoPathTargeting) {
    this.isAutoPathTargeting = isAutoPathTargeting;
  }
  /** Enum defining joystick axes for clearer code when handling joystick inputs. */
  private enum JoystickAxis {
    Ly,
    Lx,
    Ry,
    Rx;

    /**
     * Determines if the axis input should be flipped based on the robot's alliance color.
     *
     * @return true if the axis input is inverted, false otherwise.
     */
    public boolean isFlipped() {
      return this == Ly || this == Lx;
    }
  }
}

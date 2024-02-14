// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.motor.MotorBuilder;
import frc.bearbotics.motor.MotorPidBuilder;
import frc.bearbotics.motor.cancoder.CANCoders.CANCoderBuilder;
import frc.bearbotics.swerve.SwerveModule;
import frc.bearbotics.swerve.SwerveModule.SwerveModuleBuilder;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.SwerveCorner;
import frc.robot.constants.SwerveModuleConstants.BackLeftConstants;
import frc.robot.constants.SwerveModuleConstants.BackRightConstants;
import frc.robot.constants.SwerveModuleConstants.FrontLeftConstants;
import frc.robot.constants.SwerveModuleConstants.FrontRightConstants;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.stream.DoubleStream;

/** Controls the four swerve modules for autonomous and teleoperated modes. */
public class DriveSubsystem extends SubsystemBase {
  // Linked to maintain order.
  private final LinkedHashMap<SwerveCorner, SwerveModule> swerveModules = new LinkedHashMap<>();
  private final Pigeon2 pigeonImu = new Pigeon2(RobotConstants.PIGEON_CAN_ID);

  private final SwerveDriveOdometry odometry;
  private GenericEntry competitionTabMaxSpeedEntry;

  private double maxSpeed = DriveConstants.DRIVE_VELOCITY;
  private boolean fieldRelativeMode = true;

  private SwerveModuleState[] desiredSwerveModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  public DriveSubsystem() {
    for (SwerveCorner corner : SwerveCorner.values()) {
      swerveModules.put(
          corner,
          new SwerveModule(getSwerveConfigForCorner(corner), DriveConstants.DRIVE_SYSTEM_TAB));
    }

    odometry =
        new SwerveDriveOdometry(
            RobotConstants.DRIVE_KINEMATICS, getHeading(), getModulePositions());

    zeroHeading();
    setupShuffleboardTab();
  }

  public void zeroHeading() {
    pigeonImu.reset();
  }

  private void setupShuffleboardTab() {
    competitionTabMaxSpeedEntry =
        DriveConstants.COMPETITION_TAB
            .add("Maximum Drive Speed", maxSpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(2, 1)
            .withPosition(5, 1)
            .withProperties(Map.of("min", 0, "max", maxSpeed * 2))
            .getEntry();

    DriveConstants.COMPETITION_TAB.addNumber("Pigeon Heading", () -> getHeading().getDegrees());
    DriveConstants.DRIVE_SYSTEM_TAB.addBoolean("Field Relative?", () -> fieldRelativeMode);
    DriveConstants.DRIVE_SYSTEM_TAB.addDoubleArray(
        "MeasuredStates", this::getMeasuredSwerveModuleStates);
    DriveConstants.DRIVE_SYSTEM_TAB.addDoubleArray(
        "DesiredStates", this::getDesiredSwerveModuleStates);
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    maxSpeed = competitionTabMaxSpeedEntry.getDouble(DriveConstants.MAX_VELOCITY);

    for (SwerveModule module : swerveModules.values()) {
      module.updateDataLogs();
    }
  }

  private SwerveModuleBuilder getSwerveConfigForCorner(SwerveCorner corner) {
    switch (corner) {
      case FRONT_LEFT:
        return getFrontLeftSwerveConfig();
      case BACK_LEFT:
        return getBackLeftSwerveConfig();
      case FRONT_RIGHT:
        return getFrontRightSwerveConfig();
      case BACK_RIGHT:
        return getBackRightSwerveConfig();
      default:
        throw new IllegalArgumentException("Unknown corner: " + corner);
    }
  }

  private SwerveModuleBuilder getFrontLeftSwerveConfig() {
    MotorPidBuilder driveMotorPid =
        new MotorPidBuilder()
            .withP(FrontLeftConstants.DriveMotor.MotorPid.P)
            .withFf(FrontLeftConstants.DriveMotor.MotorPid.Ff);

    MotorPidBuilder pivotMotorPid =
        new MotorPidBuilder()
            .withP(FrontLeftConstants.PivotMotor.MotorPid.P)
            .withPositionPidWrappingEnabled(
                FrontLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .withPositionPidWrappingMin(
                FrontLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .withPositionPidWrappingMax(
                FrontLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    CANCoderBuilder pivotCanCoderEncoder =
        new CANCoderBuilder()
            .setId(FrontLeftConstants.PivotMotor.ABSOLUTE_ENCODER_PORT)
            .setOffsetDegrees(FrontLeftConstants.PivotMotor.ABSOLUTE_ENCODER_OFFSET);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .withName(FrontLeftConstants.DriveMotor.NAME)
            .withMotorPort(FrontLeftConstants.DriveMotor.MOTOR_PORT)
            .withCurrentLimit(FrontLeftConstants.DriveMotor.CURRENT_LIMT)
            .withMotorInverted(FrontLeftConstants.DriveMotor.INVERTED)
            .withEncoderInverted(FrontLeftConstants.DriveMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(FrontLeftConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(FrontLeftConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .withName(FrontLeftConstants.PivotMotor.NAME)
            .withMotorPort(FrontLeftConstants.PivotMotor.MOTOR_PORT)
            .withCanCoderBuilder(pivotCanCoderEncoder)
            .withCurrentLimit(FrontLeftConstants.PivotMotor.CURRENT_LIMT)
            .withMotorInverted(FrontLeftConstants.PivotMotor.INVERTED)
            .withEncoderInverted(FrontLeftConstants.PivotMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(FrontLeftConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(FrontLeftConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(FrontLeftConstants.MODULE_NAME)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getBackLeftSwerveConfig() {
    MotorPidBuilder driveMotorPid =
        new MotorPidBuilder()
            .withP(BackLeftConstants.DriveMotor.MotorPid.P)
            .withFf(BackLeftConstants.DriveMotor.MotorPid.Ff);

    MotorPidBuilder pivotMotorPid =
        new MotorPidBuilder()
            .withP(BackLeftConstants.PivotMotor.MotorPid.P)
            .withPositionPidWrappingEnabled(
                BackLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .withPositionPidWrappingMin(
                BackLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .withPositionPidWrappingMax(
                BackLeftConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    CANCoderBuilder pivotCanCoderEncoder =
        new CANCoderBuilder()
            .setId(BackLeftConstants.PivotMotor.ABSOLUTE_ENCODER_PORT)
            .setOffsetDegrees(BackLeftConstants.PivotMotor.ABSOLUTE_ENCODER_OFFSET);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .withName(BackLeftConstants.DriveMotor.NAME)
            .withMotorPort(BackLeftConstants.DriveMotor.MOTOR_PORT)
            .withCurrentLimit(BackLeftConstants.DriveMotor.CURRENT_LIMT)
            .withMotorInverted(BackLeftConstants.DriveMotor.INVERTED)
            .withEncoderInverted(BackLeftConstants.DriveMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(BackLeftConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(BackLeftConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .withName(BackLeftConstants.PivotMotor.NAME)
            .withMotorPort(BackLeftConstants.PivotMotor.MOTOR_PORT)
            .withCanCoderBuilder(pivotCanCoderEncoder)
            .withCurrentLimit(BackLeftConstants.PivotMotor.CURRENT_LIMT)
            .withMotorInverted(BackLeftConstants.PivotMotor.INVERTED)
            .withEncoderInverted(BackLeftConstants.PivotMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(BackLeftConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(BackLeftConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(BackLeftConstants.MODULE_NAME)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getFrontRightSwerveConfig() {
    MotorPidBuilder driveMotorPid =
        new MotorPidBuilder()
            .withP(FrontRightConstants.DriveMotor.MotorPid.P)
            .withFf(FrontRightConstants.DriveMotor.MotorPid.Ff);

    MotorPidBuilder pivotMotorPid =
        new MotorPidBuilder()
            .withP(FrontRightConstants.PivotMotor.MotorPid.P)
            .withPositionPidWrappingEnabled(
                FrontRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .withPositionPidWrappingMin(
                FrontRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .withPositionPidWrappingMax(
                FrontRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    CANCoderBuilder pivotCanCoderEncoder =
        new CANCoderBuilder()
            .setId(FrontRightConstants.PivotMotor.ABSOLUTE_ENCODER_PORT)
            .setOffsetDegrees(FrontRightConstants.PivotMotor.ABSOLUTE_ENCODER_OFFSET);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .withName(FrontRightConstants.DriveMotor.NAME)
            .withMotorPort(FrontRightConstants.DriveMotor.MOTOR_PORT)
            .withCurrentLimit(FrontRightConstants.DriveMotor.CURRENT_LIMT)
            .withMotorInverted(FrontRightConstants.DriveMotor.INVERTED)
            .withEncoderInverted(FrontRightConstants.DriveMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(FrontRightConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(FrontRightConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .withName(FrontRightConstants.PivotMotor.NAME)
            .withMotorPort(FrontRightConstants.PivotMotor.MOTOR_PORT)
            .withCanCoderBuilder(pivotCanCoderEncoder)
            .withCurrentLimit(FrontRightConstants.PivotMotor.CURRENT_LIMT)
            .withMotorInverted(FrontRightConstants.PivotMotor.INVERTED)
            .withEncoderInverted(FrontRightConstants.PivotMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(FrontRightConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(FrontRightConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(FrontRightConstants.MODULE_NAME)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  private SwerveModuleBuilder getBackRightSwerveConfig() {
    MotorPidBuilder driveMotorPid =
        new MotorPidBuilder()
            .withP(BackRightConstants.DriveMotor.MotorPid.P)
            .withFf(BackRightConstants.DriveMotor.MotorPid.Ff);

    MotorPidBuilder pivotMotorPid =
        new MotorPidBuilder()
            .withP(BackRightConstants.PivotMotor.MotorPid.P)
            .withPositionPidWrappingEnabled(
                BackRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_ENABLED)
            .withPositionPidWrappingMin(
                BackRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MIN)
            .withPositionPidWrappingMax(
                BackRightConstants.PivotMotor.MotorPid.POSITION_PID_WRAPPING_MAX);

    CANCoderBuilder pivotCanCoderEncoder =
        new CANCoderBuilder()
            .setId(BackRightConstants.PivotMotor.ABSOLUTE_ENCODER_PORT)
            .setOffsetDegrees(BackRightConstants.PivotMotor.ABSOLUTE_ENCODER_OFFSET);

    MotorBuilder driveConfig =
        new MotorBuilder()
            .withName(BackRightConstants.DriveMotor.NAME)
            .withMotorPort(BackRightConstants.DriveMotor.MOTOR_PORT)
            .withCurrentLimit(BackRightConstants.DriveMotor.CURRENT_LIMT)
            .withMotorInverted(BackRightConstants.DriveMotor.INVERTED)
            .withEncoderInverted(BackRightConstants.DriveMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(BackRightConstants.DriveMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(BackRightConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(driveMotorPid);

    MotorBuilder pivotConfig =
        new MotorBuilder()
            .withName(BackRightConstants.PivotMotor.NAME)
            .withMotorPort(BackRightConstants.PivotMotor.MOTOR_PORT)
            .withCanCoderBuilder(pivotCanCoderEncoder)
            .withCurrentLimit(BackRightConstants.PivotMotor.CURRENT_LIMT)
            .withMotorInverted(BackRightConstants.PivotMotor.INVERTED)
            .withEncoderInverted(BackRightConstants.PivotMotor.ENCODER_INVERTED)
            .withPositionConversionFactor(BackRightConstants.PivotMotor.POSITION_CONVERSION_FACTOR)
            .withVelocityConversionFactor(BackRightConstants.PivotMotor.VELOCITY_CONVERSION_FACTOR)
            .withMotorPid(pivotMotorPid);

    SwerveModuleBuilder moduleConfig =
        new SwerveModuleBuilder()
            .setModuleName(BackRightConstants.MODULE_NAME)
            .setDriveMotor(driveConfig)
            .setPivotMotor(pivotConfig);

    return moduleConfig;
  }

  /**
   * Get an array of swerve modules in order.
   *
   * <p>This order is detemrined by {@link SwerveCorner}
   *
   * @return An array containing the swerve modules, ordered.
   */
  public Collection<SwerveModule> getSwerveModules() {
    return swerveModules.values();
  }

  /**
   * Multiplies the maximum speed for the robot based on the specified speed mode.
   *
   * @param mode The specified speed mode set by {@link SpeedMode}.
   */
  public void setSpeedMode(SpeedMode mode) {
    competitionTabMaxSpeedEntry.setDouble(mode.getMaxSpeed());
  }

  /**
   * Sets whether the robot's movement is interpreted as field-relative or robot-relative.
   *
   * <p>{@code true} to enable field-relative mode, where the robot's movement is based on the
   * field's coordinates. {@code false} for robot-relative mode, where the robot's movement is based
   * on its own coordinates regardless of field orientation.
   *
   * @param mode Whether field-relative is enabled or not.
   */
  public void setFieldRelative(boolean mode) {
    fieldRelativeMode = mode;
  }

  /**
   * Drives the robot using joystick inputs, with the default pivot mode set to CENTER, and adjusts
   * the movement interpretation based on the current field-relative mode setting.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    drive(xSpeed, ySpeed, rot, fieldRelativeMode);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = DriveConstants.X_ACCELERATION_LIMITER.calculate(xSpeed) * maxSpeed;
    ySpeed = DriveConstants.Y_ACCELERATION_LIMITER.calculate(ySpeed) * maxSpeed;

    rot =
        DriveConstants.TURNING_ACCELERATION_LIMITER.calculate(rot)
            * DriveConstants.MAX_ANGULAR_ACCELERATION_PER_SECOND;

    if (maxSpeed == SpeedMode.TURTLE.getMaxSpeed()) {
      rot /= 18;
    } else if (maxSpeed == SpeedMode.TURBO.getMaxSpeed()) {
      rot /= 4;
    } // TODO: refactor. Maybe make maxSpeed into a SpeedMode enum and handle logic within?

    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] swerveModuleStates =
        RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.CYCLE_TIME));

    setModuleStates(swerveModuleStates);
  }

  /** Stops all drive motors. */
  public Command getDriveStopCommand() {
    return new InstantCommand(() -> drive(0, 0, 0));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param swerveModuleStates The desired swerve module states, ordered.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    Iterator<SwerveModuleState> stateIterator = Arrays.asList(swerveModuleStates).iterator();
    this.desiredSwerveModuleStates = swerveModuleStates;

    for (SwerveModule module : getSwerveModules()) {
      module.set(stateIterator.next());
    }
  }

  /**
   * Returns the state of every swerve module.
   *
   * @return The states.
   */
  public SwerveModuleState[] getModuleStates() {
    return getSwerveModules().stream()
        .map(module -> module.getState())
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * Returns the position of every swerve module.
   *
   * @return The positions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return getSwerveModules().stream()
        .map(module -> module.getPosition())
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Gets an array which contains current swerve modules rotation and velocity. This is used for
   * AdvantageScope.
   *
   * @return The array.
   */
  private double[] getMeasuredSwerveModuleStates() {
    return getNormalizedSwerveModuleStates(getModuleStates());
  }

  /**
   * Gets an array which contains the targeted swerve modules rotation and velocity. This is used
   * for AdvantageScope.
   *
   * @return The array.
   */
  private double[] getDesiredSwerveModuleStates() {
    return getNormalizedSwerveModuleStates(desiredSwerveModuleStates);
  }

  /**
   * Normalizes an array of {@link SwerveModuleState} to an array containing the states's rotation
   * and velocity. This method is used for AdvantageScope.
   *
   * @param swerveModuleStates The array of swerve module states to be normalized.
   * @return The normalized array.
   */
  private double[] getNormalizedSwerveModuleStates(SwerveModuleState[] states) {
    return Arrays.stream(states)
        .flatMapToDouble(
            state -> DoubleStream.of(state.angle.getRadians(), state.speedMetersPerSecond))
        .toArray();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(
        MathUtil.inputModulus(pigeonImu.getRotation2d().getDegrees(), 0, 360));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
    // pigeon2.addYaw(pose.getRotation().getDegrees());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return RobotConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    setModuleStates(RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }
}

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bearbotics.swerve.MotorConfig;
import frc.bearbotics.swerve.MotorConfig.MotorBuilder;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.manipulator.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeMotorEncoder;

  public IntakeSubsystem() {
    MotorBuilder intakeMotorConfig =
        new MotorBuilder()
            .setModuleName(IntakeConstants.Motor.MODULE_NAME)
            .setMotorPort(IntakeConstants.Motor.MOTOR_PORT)
            .setMotorInverted(IntakeConstants.Motor.INVERTED)
            .setCurrentLimit(IntakeConstants.Motor.CURRENT_LIMT);

    intakeMotor =
        new CANSparkMax(intakeMotorConfig.getMotorPort(), CANSparkLowLevel.MotorType.kBrushless);
    intakeMotorEncoder = intakeMotor.getEncoder();

    MotorConfig.fromMotorConstants(intakeMotor, intakeMotorEncoder, intakeMotorConfig)
        .configureMotor()
        .configureEncoder(Rotation2d.fromRotations(0))
        .burnFlash();

    setupShuffleboardTab();
  }

  public void setupShuffleboardTab() {
    DriveConstants.MANIPULATOR_SYSTEM_TAB.addDouble(
        "Intake Velocity", intakeMotorEncoder::getVelocity);
  }

  public void set(double speed) {
    intakeMotor.set(speed);
  }
}

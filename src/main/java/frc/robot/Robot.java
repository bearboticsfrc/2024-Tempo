// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.bearbotics.fms.AllianceColor;
import java.util.Optional;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    DriverStation.silenceJoystickConnectionWarning(true);

    robotContainer = new RobotContainer();
    robotContainer.robotInit();
  }

  @Override
  public void robotPeriodic() {
    Optional<Alliance> maybeAlliance = DriverStation.getAlliance();

    if (maybeAlliance.isPresent()) {
      AllianceColor.setAllianceColor(maybeAlliance.get());
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    robotContainer.setTeleop(false);
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    robotContainer.setTeleop(true);
    robotContainer.teleopInit();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void disabledInit() {
    robotContainer.disabledInit();
    robotContainer.setTeleop(false);
  }
}

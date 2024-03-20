// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.bearbotics.fms.AllianceColor;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
    updateAllianceColor();

    robotContainer = new RobotContainer();
    robotContainer.robotInit();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    updateAllianceColor();

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

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    updateAllianceColor();
  }

  @Override
  public void disabledInit() {
    robotContainer.disabledInit();
    robotContainer.setTeleop(false);
    updateAllianceColor();
  }

  private void updateAllianceColor() {
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      AllianceColor.setAllianceColor(DriverStation.getAlliance().get());
    }
  }
}

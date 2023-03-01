// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class TestAutoBalanceCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Gyro m_gyro;
  private boolean toggle;
  private int inc;

  /** Creates a new AutoBalanceCommand. */
  public TestAutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrainSubsystem);
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toggle = false;
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = m_gyro.getPitch();

    if (Math.abs(pitchAngleDegrees) < 3 && !toggle) {
      m_drivetrainSubsystem.drive(1, 0, 0, false);
    } else {
      toggle = true;
    }

    if (toggle) {
      m_drivetrainSubsystem.drive(0.5, 0, 0, false);
      inc++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return inc > 56;
    }
}

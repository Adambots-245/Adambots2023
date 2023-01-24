// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Constants.DriveConstants;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceCommand extends CommandBase {
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Gyro m_gyro;
  private boolean autoBalanceXMode;
  private boolean autoBalanceYMode;

  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrainSubsystem);
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double pitchAngleDegrees = m_gyro.getPitch();
    double rollAngleDegrees = m_gyro.getRoll();
    double xAxisRate = 0.0;
    double yAxisRate = 0.0;

    if (!autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) >= Math.abs(DriveConstants.kOffBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = true;
    } else if (autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) <= Math.abs(DriveConstants.kOonBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = false;
    }
    if (!autoBalanceYMode &&
        (Math.abs(pitchAngleDegrees) >= Math.abs(DriveConstants.kOffBalanceAngleThresholdDegrees))) {
      autoBalanceYMode = true;
    } else if (autoBalanceYMode &&
        (Math.abs(pitchAngleDegrees) <= Math.abs(DriveConstants.kOonBalanceAngleThresholdDegrees))) {
      autoBalanceYMode = false;
    }

    // Control drive system automatically,
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle

    if (autoBalanceXMode) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians) * -1;
    }
    if (autoBalanceYMode) {
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(rollAngleRadians) * -1;
    }

    m_drivetrainSubsystem.drive(xAxisRate, yAxisRate, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // TODO: Instead of stopping, perform Hockey Stop (45 degree wheels)
    m_drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (!autoBalanceXMode && !autoBalanceYMode)
      return true;
    else
      return false;
  }
}

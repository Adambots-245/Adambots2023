// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.sensors.Lidar;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LidarTurnToObjectCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  Lidar lidar;
  int debounce;

  public LidarTurnToObjectCommand(DrivetrainSubsystem drivetrainSubsystem, Lidar lidar) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    this.lidar = lidar;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    if (lidar.getInches() > 95) {
      drivetrainSubsystem.drive(0, 0, 0.15, false);
    } else {
      debounce++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return debounce >= 3;
  }
}

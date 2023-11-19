// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants;
import com.adambots.sensors.Lidar;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveToGamePieceCommand extends Command {
  private DrivetrainSubsystem drivetrainSubsystem;
  private GrabbyLifterSubsystem grabbyLifterSubsystem;
  private Lidar lidar;
  private int inc;

  /** Creates a new DriveToDistanceCommand. */
  public DriveToGamePieceCommand(DrivetrainSubsystem drivetrainSubsystem, Lidar lidar, GrabbyLifterSubsystem grabbyLifterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.grabbyLifterSubsystem = grabbyLifterSubsystem;
    this.lidar = lidar;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lidar.getInches() < 55) {
      drivetrainSubsystem.drive((lidar.getInches()-4)*0.015, 0, 0, false);
    }

    if (lidar.getInches() <= 9.5 && grabbyLifterSubsystem.getEncoder() <= Constants.GrabbyConstants.groundLifterValue+4) {
      inc++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
    if (interrupted) {
      System.out.println("DriveToDistanceCommand Interrupted");
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  inc > 5 || lidar.getInches() >= 55;
  }
}

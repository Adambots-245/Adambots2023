// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.commands.autonCommands.AutonCommands.Direction;
import com.adambots.sensors.Lidar;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToGamePieceCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  Lidar lidar;
  int debounce;
  Direction dir;
  int dirInt;

 
  public TurnToGamePieceCommand(DrivetrainSubsystem drivetrainSubsystem, Lidar lidar, Direction dir) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.lidar = lidar;
    this.dir = dir;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (dir == Direction.RIGHT) {
      dirInt = -1;
    } else if (dir == Direction.LEFT) {
      dirInt = 1;
    } 
    
    Alliance allianceColor = DriverStation.getAlliance();
    if (allianceColor == Alliance.Red) {
      dirInt *= -1;
    } else if (allianceColor != Alliance.Blue) {
      System.err.println("Alliance Color not selected (TurnToGamePieceCommand)");
    }

    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    if (lidar.getInches() > 50) {
      drivetrainSubsystem.drive(0, 0, 0.2 * dirInt, false);
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
    return debounce >= 13 && lidar.getInches() > 15;
  }
}

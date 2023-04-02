// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.sensors.Lidar;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToGamePieceCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  Lidar lidar;
  int debounce;
  int dir;

  public TurnToGamePieceCommand(DrivetrainSubsystem drivetrainSubsystem, Lidar lidar, String dir) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.lidar = lidar;
    if (dir == "right") {this.dir = 1;}
    else if (dir == "left") {this.dir = -1;}
    else { System.err.println("dir not 'left' or 'right' (TurnToGamePieceCommand)");}

    String allianceColor = DriverStation.getAlliance().name();
    if (allianceColor.toLowerCase() == "red") {
        this.dir *= -1;
    } else if (allianceColor.toLowerCase() != "blue") {
        System.err.println("Alliance Color not selected (TurnToGamePieceCommand)");
    }

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    if (lidar.getInches() > 90) {
      drivetrainSubsystem.drive(0, 0, 0.15 * dir, false);
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
    return debounce >= 3 && lidar.getInches() > 20;
  }
}

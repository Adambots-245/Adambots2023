// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Vision.VisionHelpers;
import com.adambots.sensors.Lidar;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToObjectCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  String pieceType;
  PIDController pid;
  double rotSpeed;
  Lidar lidar;
  int debounce;

  int direction;

  public TurnToObjectCommand(DrivetrainSubsystem drivetrainSubsystem, Lidar lidar, String pieceType) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    this.pieceType = pieceType;
    this.lidar = lidar;
    this.pid = new PIDController(0.02, 0, 0.0015);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionHelpers.setPipeline(0);
    direction = -1;
    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {

    if (!VisionHelpers.isDetectingPieces(pieceType) || (lidar.getInches() >= 50 && Math.abs(VisionHelpers.getPieceX(pieceType)) < 2.3))
    {
      drivetrainSubsystem.drive(0, 0, 0.1 * direction, false);
    }else{

    rotSpeed = pid.calculate(VisionHelpers.getPieceX(pieceType), 0);
    drivetrainSubsystem.drive(0, 0, rotSpeed, false);
   
    if (rotSpeed > 0) {
      direction = 1;
    }
    else {
      direction = -1;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(VisionHelpers.getPieceX("cube")) < 5;
    if (VisionHelpers.isDetectingPieces(pieceType) && Math.abs(VisionHelpers.getPieceX(pieceType)) < 2.3 && lidar.getInches() < 50) {
      debounce++;
    }
    return debounce > 5;
  }
}

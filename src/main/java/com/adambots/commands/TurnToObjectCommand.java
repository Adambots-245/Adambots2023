// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Vision.VisionHelpers;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToObjectCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  String pieceType;
  PIDController pid;
  double rotSpeed;

  int direction = -1;

  public TurnToObjectCommand(DrivetrainSubsystem drivetrainSubsystem, String pieceType) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    this.pieceType = pieceType;
    this.pid = new PIDController(0.01, 0, 0.005);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {

    if (!VisionHelpers.isDetectingPieces(pieceType))
    {
      drivetrainSubsystem.drive(0, 0, 0.2 * direction, false);
    }

    rotSpeed = pid.calculate(VisionHelpers.getPieceX(pieceType), 0);
    drivetrainSubsystem.drive(0, 0, rotSpeed, false);
   
    if (rotSpeed > 0) {
      direction = 1;
    }
    else {
      direction = -1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(VisionHelpers.getPieceX("cube")) < 5;
    return rotSpeed == rotSpeed && Math.abs(rotSpeed) < 0.05 && Math.abs(rotSpeed) > 0;
  }
}

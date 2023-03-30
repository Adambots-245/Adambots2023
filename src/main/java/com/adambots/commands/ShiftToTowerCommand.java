// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Vision.VisionHelpers;
import com.adambots.sensors.Lidar;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShiftToTowerCommand extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;
  PIDController pid;
  int debounce;

  public ShiftToTowerCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    this.pid = new PIDController(0.035, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    VisionHelpers.setPipeline(1);
    debounce = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {

    drivetrainSubsystem.drive(0, pid.calculate(-VisionHelpers.getTX(), 0), 0, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    VisionHelpers.setPipeline(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(VisionHelpers.getPieceX("cube")) < 5;
    if (!VisionHelpers.isDetected() || VisionHelpers.isAligned()) {
      debounce++;
    }else{
      debounce = 0;
    }
    return debounce > 5;
  }
}

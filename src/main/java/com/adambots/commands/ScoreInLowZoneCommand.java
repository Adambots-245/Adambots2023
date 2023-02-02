// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.ArmAndGrabbySubystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;

public class ScoreInLowZoneCommand extends CommandBase {
  /** Creates a new ScoreInLowZoneCommabnd. */
  ArmAndGrabbySubystem armAndGrabbySubystem;
  int speed;
  double position;
  public ScoreInLowZoneCommand(ArmAndGrabbySubystem armAndGrabbySubystem) {
    // this.speed = speed;
    this.armAndGrabbySubystem = armAndGrabbySubystem;
    addRequirements(armAndGrabbySubystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAndGrabbySubystem.ScoreInLowZone(speed);
    while (Math.abs(Constants.GrabbyConstants.lowZoneLifterValue - armAndGrabbySubystem.getArmLifterEncoder()) > 0.1 ) {
      if (armAndGrabbySubystem.getArmLifterEncoder() > Constants.GrabbyConstants.lowZoneLifterValue) {
        armAndGrabbySubystem.lowerArm(30);
      } else if (armAndGrabbySubystem.getArmLifterEncoder() < Constants.GrabbyConstants.lowZoneLifterValue) {
        armAndGrabbySubystem.raiseArm(30);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armAndGrabbySubystem.raiseArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

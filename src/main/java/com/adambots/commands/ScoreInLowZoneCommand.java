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
  double speed;
  //double position;
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
   // armAndGrabbySubystem.ScoreInLowZone(speed);
    // while (Math.abs(Constants.GrabbyConstants.lowZoneLifterValue - armAndGrabbySubystem.getArmLifterEncoder()) > 0.1 || Math.abs(Constants.GrabbyConstants.lowZoneExtenderValue - armAndGrabbySubystem.getRightExtenderEncoder()) > 0.1 || Math.abs(Constants.GrabbyConstants.lowZoneExtenderValue - armAndGrabbySubystem.getLeftExtenderEncoder()) > 0.1) {
    //   if (armAndGrabbySubystem.getArmLifterEncoder() > Constants.GrabbyConstants.lowZoneLifterValue) {
    //     if (Math.abs(Constants.GrabbyConstants.lowZoneLifterValue - armAndGrabbySubystem.getArmLifterEncoder()) > 1) {
    //       armAndGrabbySubystem.lowerArm();
    //     } else {
    //       armAndGrabbySubystem.lowerArm();
    //     }
    //   } else if (armAndGrabbySubystem.getArmLifterEncoder() < Constants.GrabbyConstants.lowZoneLifterValue) {
    //     if (Math.abs(Constants.GrabbyConstants.lowZoneLifterValue - armAndGrabbySubystem.getArmLifterEncoder()) > 1) {
    //       armAndGrabbySubystem.raiseArm();
    //     } else {
    //       armAndGrabbySubystem.raiseArm();
    //     }
    //   }

    //   if (armAndGrabbySubystem.getRightExtenderEncoder() > Constants.GrabbyConstants.lowZoneExtenderValue) {
    //     if (Math.abs(Constants.GrabbyConstants.lowZoneExtenderValue - armAndGrabbySubystem.getRightExtenderEncoder()) > 1) {
    //       armAndGrabbySubystem.extendArm();
    //     } else {
    //       armAndGrabbySubystem.extendArm();
    //     }
    //   } else if (armAndGrabbySubystem.getRightExtenderEncoder() < Constants.GrabbyConstants.lowZoneExtenderValue) {
    //     if (Math.abs(Constants.GrabbyConstants.lowZoneExtenderValue - armAndGrabbySubystem.getRightExtenderEncoder()) > 1) {
    //       armAndGrabbySubystem.retractArm();
    //     } else {
    //       armAndGrabbySubystem.retractArm();
    //     }
    //   }


    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // armAndGrabbySubystem.stopArmLifter();
    // armAndGrabbySubystem.stopArmExtender();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

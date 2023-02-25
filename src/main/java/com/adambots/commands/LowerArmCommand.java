// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.adambots.Constants;
import com.adambots.subsystems.GrabbyLifterSubsystem;

public class LowerArmCommand extends CommandBase {
  /** Creates a new LiftArmCommand. */
  double increment;
  GrabbyLifterSubsystem grabbyLifterSubsystem;
  public LowerArmCommand(GrabbyLifterSubsystem grabbyLifterSubsystem, double increment) {
    addRequirements(grabbyLifterSubsystem);
    this.grabbyLifterSubsystem = grabbyLifterSubsystem;
    this.increment = increment;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    grabbyLifterSubsystem.changeMaxSpeed(Constants.GrabbyConstants.manualLifterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabbyLifterSubsystem.manualDown(increment);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabbyLifterSubsystem.changeMaxSpeed(Constants.GrabbyConstants.lifterSpeed);
    grabbyLifterSubsystem.stopLifting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Constants.GrabbyConstants.State;
import com.adambots.subsystems.FirstExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FirstExtenderChangeStateCommand extends Command {
  
  FirstExtenderSubsystem firstExtenderSubsystem;
  State newState;

  public FirstExtenderChangeStateCommand(FirstExtenderSubsystem firstExtenderSubsystem, State newState) {
    this.firstExtenderSubsystem = firstExtenderSubsystem;
    this.newState = newState;
    addRequirements(firstExtenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    firstExtenderSubsystem.changeTarget(newState.getFirstExtendTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

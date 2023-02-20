// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Constants;
import com.adambots.subsystems.SecondExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SecondExtenderMidCubeCommand extends CommandBase {
  
  SecondExtenderSubsystem secondExtenderSubsystem;

  public SecondExtenderMidCubeCommand(SecondExtenderSubsystem secondExtenderSubsystem) {
    this.secondExtenderSubsystem = secondExtenderSubsystem;
    addRequirements(secondExtenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    secondExtenderSubsystem.changeTarget(Constants.GrabbyConstants.midCubeState.getSecondExtendTarget());
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.adambots.subsystems.SecondExtenderSubsystem;

public class RetractSecondStageCommand extends CommandBase {
  /** Creates a new LiftArmCommand. */
  SecondExtenderSubsystem secondExtenderSubsystem;
  public RetractSecondStageCommand(SecondExtenderSubsystem secondExtenderSubsystem) {
    addRequirements(secondExtenderSubsystem);
    this.secondExtenderSubsystem = secondExtenderSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    secondExtenderSubsystem.manualIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    secondExtenderSubsystem.stopExtending();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

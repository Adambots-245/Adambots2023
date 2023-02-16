// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;


import com.adambots.subsystems.GrabbySubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmGroundCommand extends CommandBase {
  /** Creates a new setArmMidCubeCommand. */
  GrabbySubsystem grabbySubsystem;
  public SetArmGroundCommand(GrabbySubsystem armAndGrabbySubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.grabbySubsystem = armAndGrabbySubsystem;
    addRequirements(armAndGrabbySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabbySubsystem.setPosition(grabbySubsystem.groundPosition);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;

public class SmartExtendArmCommand extends CommandBase {
  /** Creates a new LiftArmCommand. */
  FirstExtenderSubsystem firstExtenderSubsystem;
  SecondExtenderSubsystem secondExtenderSubsystem;

  public SmartExtendArmCommand(FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem) {
    this.firstExtenderSubsystem = firstExtenderSubsystem;
    this.secondExtenderSubsystem = secondExtenderSubsystem;

    addRequirements(firstExtenderSubsystem, secondExtenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (firstExtenderSubsystem.isMaxExtended()) {
      firstExtenderSubsystem.stopExtending();
      secondExtenderSubsystem.manualOut();
    } else {
      firstExtenderSubsystem.manualOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    firstExtenderSubsystem.stopExtending();
    secondExtenderSubsystem.stopExtending();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

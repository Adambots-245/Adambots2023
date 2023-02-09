// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.adambots.subsystems.ArmAndGrabbySubystem;

public class LiftArmCommand extends CommandBase {
  /** Creates a new LiftArmCommand. */
  ArmAndGrabbySubystem armAndGrabbySubystem;
  int speed;
  public LiftArmCommand(ArmAndGrabbySubystem armAndGrabbySubystem, int speed) {
    addRequirements(armAndGrabbySubystem);
    this.armAndGrabbySubystem = armAndGrabbySubystem;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAndGrabbySubystem.raiseArm(speed);
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

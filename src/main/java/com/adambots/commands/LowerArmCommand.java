// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.adambots.subsystems.ArmAndGrabbySubystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerArmCommand extends CommandBase {
  int speed;
  ArmAndGrabbySubystem armAndGrabbySubystem;
  public LowerArmCommand(ArmAndGrabbySubystem armAndGrabbySubystem, int speed) {
    this.speed = speed;
    addRequirements(armAndGrabbySubystem);
    this.armAndGrabbySubystem = armAndGrabbySubystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAndGrabbySubystem.lowerArm(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armAndGrabbySubystem.lowerArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

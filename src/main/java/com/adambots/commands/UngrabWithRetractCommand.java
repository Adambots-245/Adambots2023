// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.RobotMap;

public class UngrabWithRetractCommand extends CommandBase {
  GrabSubsystem grabSubsystem;
  GrabbyLifterSubsystem grabbyLifterSubsystem;
  FirstExtenderSubsystem firstExtenderSubsystem;
  SecondExtenderSubsystem secondExtenderSubsystem;

  public UngrabWithRetractCommand(GrabSubsystem grabSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem, FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem) {
    this.grabSubsystem = grabSubsystem;
    this.grabbyLifterSubsystem = grabbyLifterSubsystem;
    this.firstExtenderSubsystem = firstExtenderSubsystem;
    this.secondExtenderSubsystem = secondExtenderSubsystem;

    addRequirements(grabSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabSubsystem.ungrab();
    if (RobotMap.armRotationEncoder.getAbsolutePosition() >= GrabbyConstants.midCubeLifterValue-8 && RobotMap.armRotationEncoder.getAbsolutePosition() <= GrabbyConstants.highConeLifterValue+8) {
      Commands.waitSeconds(0.5).andThen(Commands.parallel(
          new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState),
          new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
          new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState)
      )).schedule();
    }
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

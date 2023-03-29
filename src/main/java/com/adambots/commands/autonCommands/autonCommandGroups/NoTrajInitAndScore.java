// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands.autonCommandGroups;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.RobotMap;
import com.adambots.commands.ArmLifterChangeStateCommand;
import com.adambots.commands.FirstExtenderChangeStateCommand;
import com.adambots.commands.SecondExtenderChangeStateCommand;
import com.adambots.commands.UngrabCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NoTrajInitAndScore extends SequentialCommandGroup{
  /** Creates a new AutonLeftRedPlaceCubeGrabCharge. */
  public NoTrajInitAndScore(DrivetrainSubsystem drivetrainSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem, FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem, GrabSubsystem grabSubsystem) {
    super(
    new InstantCommand(() -> RobotMap.GyroSensor.reset()),
    Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState)),
    new WaitCommand(1.4),
    new UngrabCommand(grabSubsystem),
    new WaitCommand(0.3)
    );
  }
}
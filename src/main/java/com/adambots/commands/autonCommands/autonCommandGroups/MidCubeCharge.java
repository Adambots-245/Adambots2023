// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands.autonCommandGroups;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.commands.FirstExtenderChangeStateCommand;
import com.adambots.commands.SecondExtenderChangeStateCommand;
import com.adambots.commands.autonCommands.AutoBalanceCommand;
import com.adambots.commands.autonCommands.HockeyStopCommand;
import com.adambots.commands.autonCommands.TraversePlatform;
import com.adambots.sensors.Gyro;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidCubeCharge extends SequentialCommandGroup{
  /** Creates a new AutonLeftRedPlaceCubeGrabCharge. */
  public MidCubeCharge(Gyro gyro, DrivetrainSubsystem drivetrainSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem, FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem, GrabSubsystem grabSubsystem) {
    super(
    new ParallelDeadlineGroup(new WaitCommand(14.6),
    new SequentialCommandGroup(
      new NoTrajInitAndScore(drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem),
      Commands.parallel(new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState)),
      // new InstantCommand(() -> drivetrainSubsystem.drive(-0.1, 0, 0, false)),
      new WaitCommand(1),
      new TraversePlatform(drivetrainSubsystem, gyro),
      // new TestAutoBalanceCommand(drivetrainSubsystem, gyro, grabbyLifterSubsystem),
      new AutoBalanceCommand(drivetrainSubsystem, gyro, grabbyLifterSubsystem),
      new HockeyStopCommand(drivetrainSubsystem)
    )).andThen(new HockeyStopCommand(drivetrainSubsystem)));
  }
}
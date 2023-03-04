// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands.autonCommandGroups;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.RobotMap;
import com.adambots.commands.ArmLifterChangeStateCommand;
import com.adambots.commands.FirstExtenderChangeStateCommand;
import com.adambots.commands.SecondExtenderChangeStateCommand;
import com.adambots.commands.autonCommands.AutoBalanceCommand;
import com.adambots.commands.autonCommands.AutonPickupCommand;
import com.adambots.commands.autonCommands.HockeyStopCommand;
import com.adambots.commands.autonCommands.TestAutoBalanceCommand;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;
import com.adambots.utils.Functions;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScorePickup extends SequentialCommandGroup{
  /** Creates a new AutonLeftRedPlaceCubeGrabCharge. */
  public ScorePickup(Trajectory traj1, DrivetrainSubsystem drivetrainSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem, FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem, GrabSubsystem grabSubsystem) {
    
    super(
    new AutoInitAndScoreCube(traj1, drivetrainSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem, grabSubsystem),
    Commands.parallel(new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)),
    // Commands.parallel(Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj1).andThen(new InstantCommand(() -> drivetrainSubsystem.stop())), new WaitCommand(1.75).andThen(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState))),
    Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj1).andThen(new InstantCommand(() -> drivetrainSubsystem.stop()), 
    new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState)),
    new WaitCommand(2.5),
    new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 2.5),
    new WaitCommand(0.75),
    Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState))
    );
  }
}

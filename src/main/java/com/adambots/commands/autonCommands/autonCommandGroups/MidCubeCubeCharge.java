// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands.autonCommandGroups;

import com.adambots.RobotMap;
import com.adambots.Constants.DriveConstants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.commands.ArmLifterChangeStateCommand;
import com.adambots.commands.FirstExtenderChangeStateCommand;
import com.adambots.commands.SecondExtenderChangeStateCommand;
import com.adambots.commands.UngrabCommand;
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
public class MidCubeCubeCharge extends SequentialCommandGroup{
  /** Creates a new AutonLeftRedPlaceCubeGrabCharge. */
  public MidCubeCubeCharge(Trajectory traj1, Trajectory traj2, DrivetrainSubsystem drivetrainSubsystem, GrabbyLifterSubsystem grabbyLifterSubsystem, FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem, GrabSubsystem grabSubsystem) {
    
    super(
    new InstantCommand(() -> RobotMap.GyroSensor.reset()),
    new InstantCommand(() -> DriveConstants.field.getObject("traj").setTrajectory(traj1)),
    new InstantCommand(() -> drivetrainSubsystem.resetOdometry(traj1.getInitialPose())),
    Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState)),
    new WaitCommand(1.7),
    new UngrabCommand(grabSubsystem),
    new WaitCommand(0.3),
    Commands.parallel(new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState)),
    Commands.parallel(Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj1), new WaitCommand(1.5).andThen(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.balancingState))),
    new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
    new WaitCommand(0.2),
    new AutonPickupCommand(drivetrainSubsystem, grabSubsystem, 0.8),
    new WaitCommand(0.2),
    Commands.parallel(new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.balancingState), new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState), new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState)),
    // new InstantCommand(() -> drivetrainSubsystem.resetOdometry(traj2.getInitialPose())),
    Functions.CreateSwerveControllerCommand(drivetrainSubsystem, traj2),
    new TestAutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor)
    // new AutoBalanceCommand(drivetrainSubsystem, RobotMap.GyroSensor),
    );
  }
}

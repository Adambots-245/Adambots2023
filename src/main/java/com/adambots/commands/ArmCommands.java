// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ArmCommands {
    private FirstExtenderSubsystem firstExtenderSubsystem;
    private SecondExtenderSubsystem secondExtenderSubsystem;
    private GrabbyLifterSubsystem grabbyLifterSubsystem;
    private GrabSubsystem grabSubsystem;

    public Command LiftArmCommand = new LiftArmCommand(grabbyLifterSubsystem);
    public Command LowerArmCommand = new LowerArmCommand(grabbyLifterSubsystem);

    public Command ExtendFirstStageCommand = new ExtendFirstStageCommand(firstExtenderSubsystem);
    public Command RetractFirstStageCommand = new RetractFirstStageCommand(firstExtenderSubsystem);

    public Command ExtendSecondStageCommand = new ExtendSecondStageCommand(secondExtenderSubsystem);
    public Command RetractSecondStageCommand = new RetractSecondStageCommand(secondExtenderSubsystem);

    public Command GrabCommand = new GrabCommand(grabSubsystem);
    public Command UngrabCommand = new UngrabCommand(grabSubsystem);

    public Command MidCubeCommand = Commands.parallel(
            new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midCubeState),
            new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midCubeState),
            new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midCubeState));
    public Command HighCubeCommand = Commands.parallel(
            new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState),
            new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState),
            new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState));

    public Command MidConeCommand = Commands.parallel(
            new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midConeState),
            new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midConeState),
            new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midConeState));
    public Command HighConeCommand = Commands.parallel(
            new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highConeState),
            new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highConeState),
            new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highConeState));

    public Command GroundCommand = Commands.parallel(
            new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
            new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
            new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState));
    public Command HomeCommand = Commands.parallel(
            new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState),
            new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
            new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState));

    public ArmCommands(FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem,
            GrabbyLifterSubsystem grabbyLifterSubsystem, GrabSubsystem grabSubsystem) {
        this.firstExtenderSubsystem = firstExtenderSubsystem;
        this.secondExtenderSubsystem = secondExtenderSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.grabSubsystem = grabSubsystem;
    }
}

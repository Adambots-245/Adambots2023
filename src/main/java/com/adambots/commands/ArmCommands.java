// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.subsystems.CANdleSubsystem;
import com.adambots.subsystems.FirstExtenderSubsystem;
import com.adambots.subsystems.GrabSubsystem;
import com.adambots.subsystems.GrabbyLifterSubsystem;
import com.adambots.subsystems.SecondExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** Add your docs here. */
public class ArmCommands {
    private FirstExtenderSubsystem firstExtenderSubsystem;
    private SecondExtenderSubsystem secondExtenderSubsystem;
    private GrabbyLifterSubsystem grabbyLifterSubsystem;
    private GrabSubsystem grabSubsystem;

    public ArmCommands(FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem,
            GrabbyLifterSubsystem grabbyLifterSubsystem, GrabSubsystem grabSubsystem) {
        this.firstExtenderSubsystem = firstExtenderSubsystem;
        this.secondExtenderSubsystem = secondExtenderSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.grabSubsystem = grabSubsystem;

    }

public LowerArmCommand lowerArmCommand() {
        return new LowerArmCommand(grabbyLifterSubsystem, secondExtenderSubsystem, 9999);
}

public LiftArmCommand liftArmCommand() {
        return new LiftArmCommand(grabbyLifterSubsystem, 9999);
}

private CommandBase chargeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.balancingState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState));
}

public CommandBase sideStationCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.sideStationState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.sideStationState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.sideStationState));
}

public CommandBase humanStationCommand(){
        return new ParallelCommandGroup(
        new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.humanStationState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.humanStationState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.humanStationState));
}

public CommandBase groundCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState));
}

public CommandBase highConeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highConeState));
}

public CommandBase midConeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midConeState));
}

public CommandBase midCubeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midCubeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midCubeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midCubeState));
}

public CommandBase highCubeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState));
}

public CommandBase homeCommand() {
        return new ParallelCommandGroup(
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState),
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState)
                );
}

public Command smartExtendArmCommand() {
        return new SmartExtendArmCommand(firstExtenderSubsystem, secondExtenderSubsystem);
}

public Command smartRetractArmCommand() {
        return new SmartRetractArmCommand(firstExtenderSubsystem, secondExtenderSubsystem);
}

public Command ungrabWithRetractCommand() {
        return new UngrabWithRetractCommand(grabSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem);
}

public Command grabCommand() {
        return new GrabCommand(grabSubsystem);
}
}

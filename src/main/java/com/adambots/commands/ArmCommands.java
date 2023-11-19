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
import edu.wpi.first.wpilibj2.command.Command;
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



private Command chargeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.balancingState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState));
}

public Command sideStationCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.sideStationState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.sideStationState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.sideStationState));
}

public Command humanStationConeCommand(){
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.humanStationConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.humanStationConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.humanStationConeState));
}


public Command humanStationClamp(){
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.humanStationGrab),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.humanStationGrab),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.humanStationGrab));
}

public Command humanStationCubeCommand(){
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.humanStationCubeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.humanStationCubeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.humanStationCubeState));
}

public Command groundCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState));
}

public Command highConeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highConeState));
}

public Command autonHighConeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.autonHighConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.autonHighConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.autonHighConeState));
}

public Command midConeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midConeState));
}

public Command midCubeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midCubeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midCubeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midCubeState));
}

public Command highCubeCommand() {
        return new ParallelCommandGroup(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState));
}

public Command homeCommand() {
        return new ParallelCommandGroup(
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState),
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState));
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

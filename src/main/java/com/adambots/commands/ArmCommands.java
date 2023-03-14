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
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ArmCommands {
    private FirstExtenderSubsystem firstExtenderSubsystem;
    private SecondExtenderSubsystem secondExtenderSubsystem;
    private GrabbyLifterSubsystem grabbyLifterSubsystem;
    private GrabSubsystem grabSubsystem;
    private CANdleSubsystem candle;

    public final Command LiftArmCommand;
    public final Command LowerArmCommand;
    public final Command ExtendFirstStageCommand;
    public final Command RetractFirstStageCommand;
    public final Command ExtendSecondStageCommand;
    public final Command RetractSecondStageCommand;
    public final Command GrabCommand;
    public final Command UngrabCommand;
    public final Command MidCubeCommand;
    public final Command HighCubeCommand;
    public final Command MidConeCommand;
    public final Command HighConeCommand;
    public final Command HomeCommand;
    public final Command GroundCommand;
    public final Command UngrabWithRetractCommand;
    public final Command SmartExtendArmCommand;
    public final Command SmartRetractArmCommand;
    public final Command HumanStationCommand;
    public final Command ChargeCommand;
    public final Command SideStationCommand;

    public ArmCommands(FirstExtenderSubsystem firstExtenderSubsystem, SecondExtenderSubsystem secondExtenderSubsystem,
            GrabbyLifterSubsystem grabbyLifterSubsystem, GrabSubsystem grabSubsystem, CANdleSubsystem candle) {
        this.firstExtenderSubsystem = firstExtenderSubsystem;
        this.secondExtenderSubsystem = secondExtenderSubsystem;
        this.grabbyLifterSubsystem = grabbyLifterSubsystem;
        this.grabSubsystem = grabSubsystem;

        LiftArmCommand = new LiftArmCommand(grabbyLifterSubsystem, 9999);
        LowerArmCommand = new LowerArmCommand(grabbyLifterSubsystem, 9999);
        ExtendFirstStageCommand = new ExtendFirstStageCommand(firstExtenderSubsystem);
        RetractFirstStageCommand = new RetractFirstStageCommand(firstExtenderSubsystem);
        ExtendSecondStageCommand = new ExtendSecondStageCommand(secondExtenderSubsystem);
        RetractSecondStageCommand = new RetractSecondStageCommand(secondExtenderSubsystem);
        GrabCommand = new GrabCommand(grabSubsystem);
        UngrabCommand = new UngrabCommand(grabSubsystem);
        UngrabWithRetractCommand = new UngrabWithRetractCommand(grabSubsystem, grabbyLifterSubsystem, firstExtenderSubsystem, secondExtenderSubsystem);

        SmartExtendArmCommand = new SmartExtendArmCommand(firstExtenderSubsystem, secondExtenderSubsystem);
        SmartRetractArmCommand = new SmartRetractArmCommand(firstExtenderSubsystem, secondExtenderSubsystem);

        MidCubeCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midCubeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midCubeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midCubeState));
                // new UseLedsCommand(candle, false));
        HighCubeCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highCubeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highCubeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highCubeState));
                // new UseLedsCommand(candle, false));

        MidConeCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.midConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.midConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.midConeState));
                // new UseLedsCommand(candle, false));
        HighConeCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.highConeState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.highConeState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.highConeState));
                // new UseLedsCommand(candle, false));

        GroundCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.groundState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.groundState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.groundState));
                // new UseLedsCommand(candle, false));
        HomeCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.initState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.initState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.initState));
                // new UseLedsCommand(candle, false));
        HumanStationCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.humanStationState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.humanStationState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.humanStationState));
                // new UseLedsCommand(candle, !false));
        SideStationCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.sideStationState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.sideStationState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.sideStationState));
                // new UseLedsCommand(candle, !false));

        ChargeCommand = Commands.parallel(
                new ArmLifterChangeStateCommand(grabbyLifterSubsystem, GrabbyConstants.balancingState),
                new FirstExtenderChangeStateCommand(firstExtenderSubsystem, GrabbyConstants.balancingState),
                new SecondExtenderChangeStateCommand(secondExtenderSubsystem, GrabbyConstants.balancingState));
                // new UseLedsCommand(candle, false));
    }
}

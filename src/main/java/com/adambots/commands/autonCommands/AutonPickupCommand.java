// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;
import com.adambots.subsystems.DrivetrainSubsystem;
import com.adambots.subsystems.GrabSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonPickupCommand extends CommandBase {
  
  private DrivetrainSubsystem driveTrainSubsystem;
  private GrabSubsystem grabSubsystem;
  private double timeSec;
  private double inc;

  public AutonPickupCommand(DrivetrainSubsystem driveTrainSubsystem, GrabSubsystem grabSubsystem, double timeSec) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
    this.grabSubsystem = grabSubsystem;
    this.timeSec = timeSec;
  }


  @Override
  public void initialize() {
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.drive(0.15, 0, 0, false);
    inc++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabSubsystem.grab();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inc >= timeSec*50;
  }
}

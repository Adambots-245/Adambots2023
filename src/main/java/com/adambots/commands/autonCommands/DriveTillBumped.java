// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveTillBumped extends Command {
  private DrivetrainSubsystem drivetrainSubsystem;

  /** Creates a new DriveTillBumpedCommand. */
  public DriveTillBumped(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.drive(0.45, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrainSubsystem.getBumped();
  }
}

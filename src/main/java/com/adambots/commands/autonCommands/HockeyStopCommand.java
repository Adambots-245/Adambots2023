// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class HockeyStopCommand extends Command {
  private DrivetrainSubsystem drivetrainSubsystem;
  private int inc;

  /** Creates a new AutoBalanceCommand. */
  public HockeyStopCommand(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrainSubsystem);
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inc = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.drive(0,0,0.1,false);
    inc++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return inc > 50;
    }
}

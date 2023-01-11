// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands.autonCommands;

import com.adambots.Constants;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdateSwervePIDCommand extends CommandBase {
  /** Creates a new RunIntakeCommand. */
  private DrivetrainSubsystem driveTrainSubsystem;

  public UpdateSwervePIDCommand(DrivetrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);

    this.driveTrainSubsystem = driveTrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = Preferences.getDouble(Constants.ModuleConstants.kPTurningKey, Constants.ModuleConstants.kPModuleTurningController);
    double kD = Preferences.getDouble(Constants.ModuleConstants.kDTurningKey, Constants.ModuleConstants.kDModuleTurningController);
    driveTrainSubsystem.setPIDValues(kP, 0d, kD);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

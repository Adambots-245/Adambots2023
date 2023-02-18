/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedCommand extends CommandBase { 
  //Pass a command that doesnt finish immediately, will be schedualed on initialization and canceled when timer expires
  long startTime;
  long currentTime;

  Command command;
  double durationInMillis;

    // Creates a new TimedCommand.
  public TimedCommand(Command command, double durationInMillis) {
    this.durationInMillis = durationInMillis;
    this.command = command;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = System.currentTimeMillis();
    System.out.println("Time Remaining: "+ (durationInMillis-(currentTime-startTime)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("timedcommandended. interrupted?" + interrupted);
    command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((currentTime-startTime) >= durationInMillis);
  }
}

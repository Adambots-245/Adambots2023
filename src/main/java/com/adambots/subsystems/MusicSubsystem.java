// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import java.util.Collection;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MusicSubsystem extends SubsystemBase {
  private Orchestra orchestra;
  private String[] songs;
  private int currentSelection = 0;
  private int musicLoop = 0;

  /** Creates a new MusicSubsystem. */
  public MusicSubsystem(Collection<TalonFX> motorsToUseForMusic, String[] availableSongs) {

    orchestra = new Orchestra(motorsToUseForMusic);
    this.songs = availableSongs;
  }

  public boolean loadMusic(int index) {
    if (index >= songs.length) {
      return false;
    }

    orchestra.loadMusic(songs[index]);
    currentSelection = index;

    return true;
  }

  public void playMusic(int index) {
    if (loadMusic(index)) {
      musicLoop = 10;
    }
  }

  @Override
  public void periodic() {

    // if a song is already playing, let it finish before playing
    if (musicLoop > 0) {
      --musicLoop;
      if (musicLoop == 0) {

        // scheduled play request
        orchestra.play();
      }
    }
  }
}

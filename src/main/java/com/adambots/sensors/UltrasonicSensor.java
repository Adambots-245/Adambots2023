// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class UltrasonicSensor {
    // The handle to access the sensor
    private final AnalogInput rangefinder;
    
    // The scaling factor:  distance in inches = volts returned / SCALING_FACTOR
    private final int SCALING_FACTOR = 512/5*24/23;
    
    /** Creates a new ultrasonic sensor hooked up to <code>portNumber</code> on the analog breakout.
     * @params portNumber The port number on the breakout.
     */
    public UltrasonicSensor(int portNumber){
        rangefinder = new AnalogInput(portNumber);
        rangefinder.setOversampleBits(2); // Completely arbitrary
        rangefinder.setAverageBits(5); // Ditto
    }
    
    /** Returns the distance measured in inches.  */
    public double getInches(){
        double volts = rangefinder.getAverageVoltage();
        return (double) (volts * SCALING_FACTOR);
    }
    
    /** Returns the distance measured in feet.  */
    public double getFeet(){
        return this.getInches() / 12.0;
    }
    
    public String toString(){
        // return (int)getFeet() + "' " + (int)(getInches() - (int) getFeet()*12) + "\"";
        return String.format("%d' %d\n", getFeet(), getInches());
    }
}

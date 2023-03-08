// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Dashboard utility to add values to Shuffleboard and automatically update.
 * 
 * Dash.add("Speed", () -> {
 *  return speed;
 * });
 */
public class Dash {

    private Dash(){
        throw new UnsupportedOperationException("Not meant to be instantiated. Utility class");
    }

    public static void add(String name, DoubleSupplier dval){
        Shuffleboard.getTab("debug").addDouble(name, dval);
    }

    public static void add(String name, LongSupplier ival){
        Shuffleboard.getTab("debug").addInteger(name, ival);
    }
    
    public static void add(String name, BooleanSupplier bval){
        Shuffleboard.getTab("debug").addBoolean(name, bval);
    }
}

package com.adambots.utils;

import java.util.ArrayList;
import java.util.Date;
import java.util.Random;
import java.util.logging.Level;

import com.adambots.Constants.ModuleConstants;
import com.revrobotics.ColorMatch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public class Test {

    public static void main(String[] args) {
        turn (45, Units.degreesToRadians(180));
    }

    public static void main2(String[] args) {
        // System.out.println("Class Name: " + Log.getCallerClassName());
        // Log.saveToFile("E:/test.log");
        Log.info("Test", "Waiting for input", args.length);
        // Log.setFilter(Level.SEVERE);
        Log.info("Test2", "Waiting for input2", args.length);
        Log.infoF("Test this %% %s - %d", "log message:", args.length);

        Color color = Color.kRed;
        Log.infoF("Color: %f", color.red);

        // Logg.setLevel(Level.INFO);
        Log.info("Test This Too!");
        new Test().new innerClass().logThis();

        ArrayList<Integer> arrL = new ArrayList<Integer>(); 
        arrL.add(1); 
        arrL.add(2); 

        arrL.forEach(n -> Log.infoF("arrL: %d", n));

        // return;
        // PIDController controller = new PIDController(1.1325, 0, 0);
        // PIDController controller2 = new PIDController(1.1325, 0, 0);

        // double setpoint = 5;
        // controller.setSetpoint(setpoint);
        // controller2.setSetpoint(setpoint);
        // // controller.enableContinuousInput(-15, 15);
        // controller2.enableContinuousInput(-5, 5);

        // double measurement = 15; //degrees
        // double calculatedOutput = controller.calculate(measurement, setpoint);;
        // double calculatedOutput2 = controller2.calculate(measurement, setpoint);;

        // StringBuilder sb1 = new StringBuilder();
        // StringBuilder sb2 = new StringBuilder();
        
        // sb1.append(measurement + ":" + controller.getPositionError() + ":" + calculatedOutput + "\n");
        // sb2.append(measurement + ":" + controller2.getPositionError() + ":" + calculatedOutput2 + "\n");
        
        // int count = 0;
        // Random rand = new Random();

        // while (!controller.atSetpoint()){

        //     double randomError = (rand.nextDouble() * (0.005 - 0.001)) + 0.001; 

        //     measurement = measurement + Math.signum(calculatedOutput) * (0.325 + randomError);
        //     calculatedOutput = controller.calculate(measurement, setpoint);
        //     calculatedOutput2 = controller2.calculate(measurement, setpoint);

        //     sb1.append(measurement + ":" + controller.getPositionError() + ":" + calculatedOutput + "\n");
        //     sb2.append(measurement + ":" + controller.getPositionError() + ":" + calculatedOutput2 + "\n");

        //     if (count++ > 1000)
        //         break;
        // }

        // System.out.println("Measurement 1");
        // System.out.println(sb1.toString());
        
        // System.out.println("Measurement 2");
        // System.out.println(sb2.toString());
    }

    private final static ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          ModuleConstants.kIModuleTurningController,
          ModuleConstants.kDModuleTurningController,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    public static void turn(double angle, double currentModuleAngleInRadians) {

        m_turningPIDController.reset(currentModuleAngleInRadians);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // double currentModuleAngleInRadians = 0; 
        var radAngle = Units.degreesToRadians(45);
        double turnAngleError = Math.abs(radAngle - currentModuleAngleInRadians);
        
        var swerveModuleStates = new SwerveModuleState(0, new Rotation2d(radAngle));
        var desiredState = SwerveModuleState.optimize(swerveModuleStates, new Rotation2d(currentModuleAngleInRadians));
        double pidOut = m_turningPIDController.calculate(currentModuleAngleInRadians, desiredState.angle.getRadians());
        // double pidOut = m_turningPIDController.calculate(0.785, 0.785);
    
        // if robot is not moving, stop the turn motor oscillating
        // if (turnAngleError < 0.5
        //     && Math.abs(getState().speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
        //   pidOut = 0;
    
        System.out.printf("PID Out: %f\n", pidOut);
      }

    public class innerClass{
        public void logThis(){
            Log.severe("Test innerClass: ");
        }
    }
}

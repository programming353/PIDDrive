// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int leftFrontMotorID = 0;
    public static int leftBackMotorID = 0;
    public static int rightFrontMotorID = 0;
    public static int rightBackMotorID = 0;

      /*PID Constants*/
      public static double kP = 0;
      public static double kI = 0;
      public static double kD = 0;
      public static double kIz = 0;
      public static double kFF = 0.000156;
      public static double kMaxOutput = 1;
      public static double kMinOutput = -1;
      public static double maxRPM = 5700;
      public static double maxVel = 2000;
      public static double maxAcc = 1500;

}

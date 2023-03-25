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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // Drive motors
  public static final int LEFT_FRONT = 1;
  public static final int LEFT_BACK = 2;

  public static final int RIGHT_FRONT = 3;
  public static final int RIGHT_BACK = 4;



  public static final double LEFT_ANGLE_P = 0;
  public static final double RIGHT_ANGLE_P = 0;

  //velocity PIDs
  public static final double VEL_L_P = 0.0339;// 0.037 | 0.035 - 0.036
  public static final double VEL_L_I = 0.00;
  public static final double VEL_L_D = 0.00;

  public static final double VEL_R_P = 0.0352; // 0.09
  public static final double VEL_R_I = 0.00;
  public static final double VEL_R_D = 0.00;

  public static final double MAXSPEED = 100; // inches per second

  public static final double TURN_P = 0.01;
public static final double TURN_I = 0;
public static final double TURN_D = 0;
}

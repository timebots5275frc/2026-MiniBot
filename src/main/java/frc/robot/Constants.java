// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.CustomTypes.PID;

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

  public static class DriveConstants {
    public static final PID DRIVE_PID = new PID(0.0001,0,0.000001); //TODO: PID TUNING
    public static final int FORWARD = 1; //flip sign to flip foward

    public static final int LEFT_MOTOR_ID = 2; //TODO: set in phoenix tuner
    public static final int RIGHT_MOTOR_ID = 3; //TODO: same as above
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 5;

    public static final int DRIVE_FREE_LIMIT = 35; 
    public static final int DRIVE_STALL_LIMIT = 60; 


    public static final double MAX_X_SPEED = 2; // meters per second
    public static final double MAX_Z_ROTATION = 360; // degrees per second


    public static final double TRACK_WIDTH = 16 * MathConstants.INCH_TO_METER; //TODO: the distance between the middle of left and right wheel
    public static final double GEAR_RATIO = 10.71; //TODO: how many rotations of the motor rotate the wheel
    public static final double WHEEL_DIAMETER = 4 * MathConstants.INCH_TO_METER; //TODO:
    public static final double ROBOT_MASS = 45;
    public static final double ROBOT_LENGTH = 20 * MathConstants.INCH_TO_METER ; //TODO: distance from front to back of robot
    public static final double MOI = (1.0 / 12.0) * Constants.DriveConstants.ROBOT_MASS * (Math.pow(Constants.DriveConstants.TRACK_WIDTH, 2) + Math.pow(Constants.DriveConstants.ROBOT_LENGTH, 2)); // moment of inertia
    

    public static final double metersToRotations = Constants.DriveConstants.GEAR_RATIO / (Math.PI * Constants.DriveConstants.WHEEL_DIAMETER);
  }

  public static class ControllerConstants {

    public static final double DEADZONE_DRIVE = 0.05;
    public static final double DEADZONE_STEER = 0.05;

  }

   public static final class MathConstants {
    public static final double INCH_TO_METER = 0.0254;
  }
}

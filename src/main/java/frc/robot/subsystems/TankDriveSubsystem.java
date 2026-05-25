// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDriveSubsystem extends SubsystemBase {

    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private SparkClosedLoopController leftMotorController;
    private SparkClosedLoopController righMotorController;

    

  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem() {
    initMotors();
  }

  private void initMotors() {

    Constants.DriveConstants.DRIVE_PID.setFreeLimit(Constants.DriveConstants.DRIVE_FREE_LIMIT);
    Constants.DriveConstants.DRIVE_PID.setStallLimit(Constants.DriveConstants.DRIVE_STALL_LIMIT);
    

    leftMotor = new SparkMax(Constants.DriveConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(leftMotor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    leftMotorController = leftMotor.getClosedLoopController();  

    rightMotor = new SparkMax(Constants.DriveConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(rightMotor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    righMotorController = rightMotor.getClosedLoopController(); 

  }

  /**
   *  
   * @param xSpeed 
   * meters per second
   * @param zRotation
   * degrees per second, positive is Counter Clockwise
   */
    public void arcadeDrive(double xSpeed, double zRotation) {

      xSpeed = Math.min(xSpeed, Constants.DriveConstants.MAX_X_SPEED);
      xSpeed = Math.max(xSpeed, -Constants.DriveConstants.MAX_X_SPEED);

      zRotation = Math.min(zRotation, Constants.DriveConstants.MAX_Z_ROTATIONS);
      zRotation = Math.max(zRotation, -Constants.DriveConstants.MAX_Z_ROTATIONS);

      double zRotationMetersPerSecond = Math.toRadians(zRotation) * (Constants.DriveConstants.TRACK_WIDTH / 2);

      double leftSpeed = xSpeed - zRotationMetersPerSecond;
      double rightSpeed = xSpeed + zRotationMetersPerSecond;

      double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotationMetersPerSecond));
      double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotationMetersPerSecond));
      if (greaterInput == 0.0) {
        leftSpeed = 0;
        rightSpeed = 0;
      } else {
        double saturatedInput = (greaterInput + lesserInput) / greaterInput;
        leftSpeed /= saturatedInput;
        rightSpeed /= saturatedInput;
      }
      
      wheelDrive(leftSpeed, rightSpeed);
    }

   /**
 * meters per second
 */
  private void wheelDrive(double leftSpeed, double rightSpeed) {
      //60 - converts seconds to minutes;
      //(Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)) gets the wheel circumfrence, otherwise meters per rotation
      //divide by the gear ratio to convert to wheel rotations
      
      double conversionFactor = (60 / (Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)) / Constants.DriveConstants.GEAR_RATIO;

      double leftRPM = leftSpeed * conversionFactor;
      double rightRPM = rightSpeed * conversionFactor;

      leftMotorController.setReference(leftRPM, ControlType.kVelocity);
      righMotorController.setReference(rightRPM, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

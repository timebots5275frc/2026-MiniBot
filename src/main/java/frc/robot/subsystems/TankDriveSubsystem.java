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
   * input a value 0.0 to 1.0 
   * 0.0 0% Current
   * 1.0 100% Current
   * @param left 
   * @param right
   */
  public void drive(double left, double right) {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

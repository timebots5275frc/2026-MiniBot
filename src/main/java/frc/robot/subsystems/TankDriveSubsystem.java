// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDriveSubsystem extends SubsystemBase {

    private SparkMax leftLeaderMotor;
    private SparkMax rightLeaderMotor;

    private SparkMax leftFollowerMotor;
    private SparkMax rightFollowerMotor;

    private SparkClosedLoopController leftMotorController;
    private SparkClosedLoopController rightMotorController;

    private DifferentialDrivetrainSim driveSim;

    

  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem() {
    initMotors();
    initSim();
  }

  private void initMotors() {

    Constants.DriveConstants.DRIVE_PID.setFreeLimit(Constants.DriveConstants.DRIVE_FREE_LIMIT);
    Constants.DriveConstants.DRIVE_PID.setStallLimit(Constants.DriveConstants.DRIVE_STALL_LIMIT);
    

    leftLeaderMotor = new SparkMax(Constants.DriveConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(leftLeaderMotor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    leftMotorController = leftLeaderMotor.getClosedLoopController();  

    rightLeaderMotor = new SparkMax(Constants.DriveConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(rightLeaderMotor,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    rightMotorController = rightLeaderMotor.getClosedLoopController(); 

    //Followers
    leftFollowerMotor = new SparkMax(Constants.DriveConstants.LEFT_FOLLOWER_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config1 = Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(leftLeaderMotor, IdleMode.kBrake);
    config1.follow(leftLeaderMotor, false); 
    leftFollowerMotor.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightFollowerMotor = new SparkMax(Constants.DriveConstants.RIGHT_FOLLOWER_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config2 = Constants.DriveConstants.DRIVE_PID.setSparkMaxPID(rightLeaderMotor, IdleMode.kBrake);
    config2.follow(rightLeaderMotor, false); 
    rightFollowerMotor.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  private void initSim() {

    LinearSystem<N2, N2, N2> drivetrainPlant = LinearSystemId.createDrivetrainVelocitySystem(
    DCMotor.getNeo550(2),
    Constants.DriveConstants.ROBOT_MASS,
    Constants.DriveConstants.WHEEL_DIAMETER / 2,
    Constants.DriveConstants.TRACK_WIDTH / 2,
    Constants.DriveConstants.MOI,
    Constants.DriveConstants.GEAR_RATIO
    );

    driveSim = new DifferentialDrivetrainSim(null, DCMotor.getNeo550(2), Constants.DriveConstants.GEAR_RATIO, Constants.DriveConstants.TRACK_WIDTH, Constants.DriveConstants.WHEEL_DIAMETER/2, null);
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

      zRotation = Math.min(zRotation, Constants.DriveConstants.MAX_Z_ROTATION);
      zRotation = Math.max(zRotation, -Constants.DriveConstants.MAX_Z_ROTATION);

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

      double conversionFactor = (60 / (Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)) * Constants.DriveConstants.GEAR_RATIO;

      double leftRPM = leftSpeed * conversionFactor;
      double rightRPM = rightSpeed * conversionFactor;

      leftMotorController.setReference(leftRPM * Constants.DriveConstants.FOWARD, ControlType.kVelocity);
      rightMotorController.setReference(rightRPM * Constants.DriveConstants.FOWARD, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDriveSubsystem extends SubsystemBase {

    private SparkMax leftLeaderMotor;
    private SparkMax rightLeaderMotor;

    private SparkMax leftFollowerMotor;
    private SparkMax rightFollowerMotor;

    private SparkClosedLoopController leftMotorController;
    private SparkClosedLoopController rightMotorController;


		//Simulation
    private DifferentialDrivetrainSim driveSim;
		private Field2d field2d;

		private SparkMaxSim leftMotorSim;
		private SparkMaxSim rightMotorSim;
    

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

		leftMotorSim = new SparkMaxSim(leftLeaderMotor, DCMotor.getNeo550(2));
    rightMotorSim = new SparkMaxSim(rightLeaderMotor, DCMotor.getNeo550(2));

    LinearSystem<N2, N2, N2> drivetrainPlant = LinearSystemId.createDrivetrainVelocitySystem(
			DCMotor.getNeo550(2),
			Constants.DriveConstants.ROBOT_MASS,
			Constants.DriveConstants.WHEEL_DIAMETER / 2,
			Constants.DriveConstants.TRACK_WIDTH / 2,
			Constants.DriveConstants.MOI,
			Constants.DriveConstants.GEAR_RATIO
    );

    driveSim = new DifferentialDrivetrainSim(drivetrainPlant, DCMotor.getNeo550(2), Constants.DriveConstants.GEAR_RATIO, Constants.DriveConstants.TRACK_WIDTH, Constants.DriveConstants.WHEEL_DIAMETER/2, null);
		driveSim.setPose(new Pose2d(0,0,new Rotation2d(0)));
		field2d = new Field2d();
		field2d.setRobotPose(driveSim.getPose());
		//SmartDashboard.putData("Field", field2d);
  }

  /**
   *  
   * @param xSpeed 
   * meters per second
   * @param zRotation
   * radians per second, positive is Counter Clockwise
   */
    public void arcadeDrive(double xSpeed, double zRotation) {

			xSpeed = Math.min(xSpeed, Constants.DriveConstants.MAX_X_SPEED);
			xSpeed = Math.max(xSpeed, -Constants.DriveConstants.MAX_X_SPEED);

			zRotation = Math.min(zRotation, Constants.DriveConstants.MAX_Z_ROTATION);
			zRotation = Math.max(zRotation, -Constants.DriveConstants.MAX_Z_ROTATION);

			// Normalize both inputs to -1 to 1
			double xSpeedNorm = xSpeed / Constants.DriveConstants.MAX_X_SPEED;
			double zRotationNorm = zRotation / Constants.DriveConstants.MAX_Z_ROTATION;

			double leftSpeed = xSpeedNorm - zRotationNorm;
			double rightSpeed = xSpeedNorm + zRotationNorm;

			double greaterInput = Math.max(Math.abs(xSpeedNorm), Math.abs(zRotationNorm));
			double lesserInput = Math.min(Math.abs(xSpeedNorm), Math.abs(zRotationNorm));
			if (greaterInput == 0.0) {
					leftSpeed = 0;
					rightSpeed = 0;
			} else {
					double saturatedInput = (greaterInput + lesserInput) / greaterInput;
					leftSpeed /= saturatedInput;
					rightSpeed /= saturatedInput;
			}

			// leftSpeed and rightSpeed are now -1 to 1, scale back to m/s
			wheelDrive(leftSpeed * Constants.DriveConstants.MAX_X_SPEED, rightSpeed * Constants.DriveConstants.MAX_X_SPEED);
	}

   /**
 * meters per second
 */
  private void wheelDrive(double leftSpeed, double rightSpeed) {
      //60 - converts seconds to minutes;
      //(Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)) gets the wheel circumfrence, otherwise meters per rotation
      //divide by the gear ratio to convert to wheel rotations
			SmartDashboard.putNumber("leftSpeed", leftSpeed);
			SmartDashboard.putNumber("rightSpeed", rightSpeed);
			

      double conversionFactor = (60 / (Math.PI * Constants.DriveConstants.WHEEL_DIAMETER)) * Constants.DriveConstants.GEAR_RATIO;

      double leftRPM = leftSpeed * conversionFactor;
      double rightRPM = rightSpeed * conversionFactor;

      leftMotorController.setReference(leftRPM * Constants.DriveConstants.FORWARD, ControlType.kVelocity);
      rightMotorController.setReference(rightRPM * Constants.DriveConstants.FORWARD, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
	public void simulationPeriodic() {

		double leftVoltage = leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
		double rightVoltage = rightMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();

		driveSim.setInputs(leftVoltage, rightVoltage);
		driveSim.update(0.02);

		leftMotorSim.iterate(driveSim.getLeftVelocityMetersPerSecond() * Constants.DriveConstants.metersToRotations * 60, RobotController.getBatteryVoltage(), 0.02);
    rightMotorSim.iterate(driveSim.getRightVelocityMetersPerSecond() * Constants.DriveConstants.metersToRotations * 60, RobotController.getBatteryVoltage(), 0.02);
			
		field2d.setRobotPose(driveSim.getPose());

		SmartDashboard.putNumber("SimHeadingDeg", driveSim.getHeading().getDegrees());
		SmartDashboard.putNumber("LeftVoltage", leftVoltage);
		SmartDashboard.putNumber("RightVoltage", rightVoltage);
		SmartDashboard.putNumber("PoseX", driveSim.getPose().getX());
		SmartDashboard.putNumber("PoseY", driveSim.getPose().getY());
		SmartDashboard.putNumber("PoseRot", driveSim.getPose().getRotation().getDegrees());

		SmartDashboard.putNumberArray("RobotPose", new double[]{
				driveSim.getPose().getX(),
				driveSim.getPose().getY(),
				driveSim.getPose().getRotation().getRadians() // try Radians() if Degrees() doesn't work
		});
  }
}

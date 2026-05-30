// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.TankDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private Input input;
  private TankDriveSubsystem tankDriveSubsystem;

  /** Creates a new DriveCommand. */
  public DriveCommand(Input input, TankDriveSubsystem tankDriveSubsystem) {
    this.tankDriveSubsystem = tankDriveSubsystem;
    this.input = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tankDriveSubsystem.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = input.ControllerInput(); //1.0 to -1.0
    double zRotation = input.ControllerTurn();//1.0 to -1.0
    
    //Square inputs
    xSpeed = xSpeed * Math.abs(xSpeed);
    zRotation = zRotation * Math.abs(zRotation);

    tankDriveSubsystem.arcadeDrive(xSpeed * Constants.DriveConstants.MAX_X_SPEED, zRotation * Constants.DriveConstants.MAX_Z_ROTATION);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

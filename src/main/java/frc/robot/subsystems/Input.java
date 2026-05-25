// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CustomTypes.Math.Vector2;

public class Input extends SubsystemBase {
  
  XboxController controller;
  double controllerSpeed;
  

  double rawControllerInput = 0;
  double rawControllerTurn = 0;

  double controllerInput = 0;
  double controllerTurn = 0;

 

  public static double Throttle;

  public Input(XboxController xboxController) {
    controller = xboxController;
    
  }

  @Override
  public void periodic() {
    
    getRawControllerInput();
    calculateControllerInput();
    
  }

  void getRawControllerInput() {
    rawControllerInput = -controller.getLeftY();
    rawControllerTurn = -controller.getRightX();
  }

  void calculateControllerInput() {
    controllerInput = calculateInputWithDeadzone(rawControllerInput, Constants.ControllerConstants.DEADZONE_DRIVE);
    controllerTurn = calculateInputWithDeadzone(rawControllerTurn, Constants.ControllerConstants.DEADZONE_STEER);
  }


  public void incrementControllerSpeed() {
    if (controllerSpeed + 0.2 <= 1) { controllerSpeed += 0.2; }
  }

  public void decrementControllerSpeed() {
    if (controllerSpeed - 0.2 >= 0) { controllerSpeed -= 0.2; }
  }

  public double getControllerSpeed() {
    return controllerSpeed;
  }

  public void flipRumble() {
    System.out.println("Flip rumble");
    rumbleController(GenericHID.RumbleType.kBothRumble, 1);
  }

  public void stopRumble() {
    rumbleController(GenericHID.RumbleType.kBothRumble, 0.0);
  }

  public void rumbleController(GenericHID.RumbleType rumbleType, double Throttle) {
    controller.setRumble(rumbleType, Throttle);
  }

  public double ControllerInput() {return controllerInput; }
  public double ControllerTurn() {return controllerTurn; }

  public double calculateInputWithDeadzone(double input, double deadZone) {
    if (Math.abs(input) < deadZone) {
        return 0;
    }

    if (input > 0) {
        return (input - deadZone) / (1 - deadZone);
    } else if (input < 0) {
        return (input + deadZone) / (1 - deadZone);
    }
    return 0;
}
}
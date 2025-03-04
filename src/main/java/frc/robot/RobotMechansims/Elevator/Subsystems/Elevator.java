// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.Elevator.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BaseConstants;
import frc.robot.Configs;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotUtilities.MiscUtils;

public class Elevator extends SubsystemBase {

  // Declare the variables that will hold the objects and refrences in this class.
  private final SparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;
  private final SparkClosedLoopController elevatorCLC;

  private double prevPosition;
  private boolean rampActive;

  public enum POSITIONS {
    LOW,
    MID,
    HIGH
  }

  public Elevator() {

    // Create a new motor controller object and store it in the 'elevatorMotor'
    // variable.  We'll pass the motor controller id and the motors type into
    // the motor controller object's constructor.
    elevatorMotor = new SparkMax(MechanismConstants.elevatorValues.motorID, MotorType.kBrushless);

    // Use methods in the previously created motor controller object (which is stored 
    // in the 'elevatorMotor' variable) to get a refrence to the motor's encoder and
    // its closed loop controller.  We'll finally store those refrences in the
    // 'elevatorEncloder' and 'elevatorCLC' variables. 
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorCLC = elevatorMotor.getClosedLoopController();

    // Call the 'configure' method in the motor controller object stored in the 'elevatorMotor' variable.
    // We'll pass in the confiruation object from another file into the 'configure' method as parameters.
    elevatorMotor.configure(Configs.ElevatorConfig.elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // Set the position of the elevator using CLC.  (Contains a clamp for safety)
  public void goToPositionClamped(double position) { 

    // We're calling the 'MiscUtils.clamp' method and passing in some values and variables
    // as parameters.  We're then assigning the output of the 'MiscUtils.clamp' method
    // to the value of a new variable called 'positionClamped'.
    double positionClamped = MiscUtils.clamp(0, MechanismConstants.elevatorValues.maxHeightExtensionRotations, position);

    // Calling the 'setRefrence' method in the motor object stored in the elevatorCLC vairable.
    // We'll pass in the previously defined 'positionClamped' variable as well as
    // the type of controller that we want to use.
    REVLibError error = elevatorCLC.setReference(positionClamped, ControlType.kPosition);
    if (error != REVLibError.kOk) {
      elevatorMotor.stopMotor();
      System.out.println("Something broke!!!!!!!!!!");
    }
  }

  public void incramentUp() {
    elevatorCLC.setReference(elevatorEncoder.getPosition() + 1, ControlType.kPosition);
  }

  public void incramentDown() {
    elevatorCLC.setReference(elevatorEncoder.getPosition() - 1, ControlType.kPosition);
  }

  // Get the position of the elevator
  public double getPosition() {

    // We'll take the
    return elevatorEncoder.getPosition();

  }

  public void ramp() {
    if (BaseConstants.DriveConstants.elevatorSpeedOverride < 1) {
      BaseConstants.DriveConstants.elevatorSpeedOverride += 0.01;
    } else {
      rampActive = false;
    }
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());

    // if (elevatorEncoder.getPosition() < 50) {
    //   BaseConstants.DriveConstants.elevatorSpeedOverride = 1;
    // } else {
    //   BaseConstants.DriveConstants.elevatorSpeedOverride = 0.3;
    // }

    if (elevatorEncoder.getPosition() < 50 && prevPosition > 50) {
      rampActive = true;
    } else if (elevatorEncoder.getPosition() > 50 && prevPosition < 50) {
      BaseConstants.DriveConstants.elevatorSpeedOverride = 0.3;
    }

    if (rampActive) {
      ramp();
    }

    prevPosition = elevatorEncoder.getPosition();
  }
}

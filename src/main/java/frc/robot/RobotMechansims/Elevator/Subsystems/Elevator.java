// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.Elevator.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.RobotUtilities.MiscUtils;

public class Elevator extends SubsystemBase {

  // Constants specific to this mechanism
  private final double maxHeightExtensionRotations = 0; // Set Me Plz

  // Declare the variables to store their respective objects.
  private final SparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;
  private final SparkClosedLoopController elevatorCLC;

  /** Creates a new PrimaryElevator. */
  public Elevator(int motorId) {

    // Put a new motor object into the elevatorMotor variable.
    elevatorMotor = new SparkMax(motorId, MotorType.kBrushless);

    // Get encoder and clc and put them in their respective variables
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorCLC = elevatorMotor.getClosedLoopController();

    // Apply configuration in Configs
    elevatorMotor.configure(Configs.PrimaryElevatorConfig.elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Deprecated
/* Use goToPositionClamped instead. */
  public void goToPosition(double position) {
    elevatorCLC.setReference(position, ControlType.kDutyCycle);
  }

  // Set the position of the elevator using CLC.  (Contains a clamp for safety)
  public void goToPositionClamped(double position) {
    elevatorCLC.setReference(MiscUtils.clamp(0, maxHeightExtensionRotations, position), ControlType.kDutyCycle);
  }

  // Get the position of the elevator
  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

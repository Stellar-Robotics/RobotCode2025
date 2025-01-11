// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.PrimaryElevator.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class PrimaryElevator extends SubsystemBase {

  // Declare the variable that will hold the motor object.
  private final SparkMax elevatorMotor;

  private final RelativeEncoder elevatorEncoder;

  private final SparkClosedLoopController elevatorCLC;

  /** Creates a new PrimaryElevator. */
  public PrimaryElevator(int motorId) {

    // Put a new motor object into the elevatorMotor variable.
    elevatorMotor = new SparkMax(motorId, MotorType.kBrushless);
    // Got the encoder from another object and put it in the elevatorEncoder.
    elevatorEncoder = elevatorMotor.getEncoder();
    // Get closed loop controler
    elevatorCLC = elevatorMotor.getClosedLoopController();

    elevatorMotor.configure(Configs.PrimaryElevatorConfig.elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void goToPosition(double position) {

    elevatorCLC.setReference(position, ControlType.kDutyCycle);

  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

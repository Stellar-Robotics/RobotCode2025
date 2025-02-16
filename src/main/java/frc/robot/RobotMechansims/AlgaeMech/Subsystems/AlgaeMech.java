// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.AlgaeMech.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.RobotMechansims.MechanismConstants;

public class AlgaeMech extends SubsystemBase {

  // Declare the variables that will hold our motor controller objects.
  private final SparkMax extensionMotor;
  private final SparkMax pickupMotor;

  public AlgaeMech() {

    // Create two new motor controller objects and store them in the 'extensionMotor' and
    // 'pickupMotor' variables.  We pass in the motor controller ids and motor
    // types a parameters when creating the new motor controller objects.
    extensionMotor = new SparkMax(MechanismConstants.AlgaeMechValues.extensionMotorID, MotorType.kBrushless);
    pickupMotor = new SparkMax(MechanismConstants.AlgaeMechValues.pickupMotorID, MotorType.kBrushless);

    // We'll call the 'configure' method in each of our new motor controller objects
    // that we are storing in our aformentioned variables.  We'll pass in some configuration
    // objects from another file as parameters to the 'configure' method.
    extensionMotor.configure(Configs.AlgaeMechConfig.extensionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pickupMotor.configure(Configs.AlgaeMechConfig.pickupMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // We'll create a method that other code can call to run the intake motors.
  public void runPickup(double speed) {

    // We'll call the 'set' method in the motor controller object stored
    // in the pickupMotor variable.  We'll make sure to pass in our
    // speed parameter.
    pickupMotor.set(speed);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}

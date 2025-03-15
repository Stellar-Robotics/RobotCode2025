// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.AlgaeMech.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.RobotMechansims.MechanismConstants;

public class AlgaeMech extends SubsystemBase {

  // Declare the variable that will hold our motor controller object.
  private final SparkFlex pickupMotor;

  private final DoubleSolenoid leftSolenoid;
  private final DoubleSolenoid rightSolenoid;

  private boolean lastPos = false;

  public AlgaeMech(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {

    // Create two new motor controller objects and store them in the 'extensionMotor' and
    // 'pickupMotor' variables.  We pass in the motor controller ids and motor
    // types a parameters when creating the new motor controller objects.
    pickupMotor = new SparkFlex(MechanismConstants.AlgaeMechValues.pickupMotorID, MotorType.kBrushless);

    // We'll call the 'configure' method in each of our new motor controller objects
    // that we are storing in our aformentioned variables.  We'll pass in some configuration
    // objects from another file as parameters to the 'configure' method.
    pickupMotor.configure(Configs.AlgaeMechConfig.pickupMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Configure solenoids
    this.leftSolenoid = leftSolenoid;
    this.rightSolenoid = rightSolenoid;
  }

  // We'll create a method that other code can call to run the intake motors.
  public void setSpeed(double speed) {

    // We'll call the 'set' method in the motor controller object stored
    // in the pickupMotor variable.  We'll make sure to pass in our
    // speed parameter.
    pickupMotor.set(speed);
    
  }

  public Command toggleExtension() {
    return Commands.runOnce(() -> {
      leftSolenoid.set(!lastPos ? Value.kForward : Value.kReverse);
      rightSolenoid.set(!lastPos ? Value.kForward : Value.kReverse);
      lastPos = !lastPos;
    }, this);
  }

  public Command actuateExtension(boolean in) {
    return Commands.runOnce(() -> {
      if (in) {
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kReverse);
        lastPos = false;
      } else {
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);
        lastPos = true;      
      }
    }, this);
  }

  public Command runPickup(double speed) {
    return Commands.runOnce(() -> {
      this.setSpeed(speed);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}

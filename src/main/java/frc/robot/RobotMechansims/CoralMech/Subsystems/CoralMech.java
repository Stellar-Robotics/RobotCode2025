// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.CoralMech.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class CoralMech extends SubsystemBase {

  // Declare the variables to hold our motor controller and closed loop controller objects.
  private final SparkMax rollerMotor;
  private final SparkClosedLoopController rollerCLC;


  public CoralMech(int motorID) {

    // Create a new motor controller object and store it in the rollerMotor variable.
    rollerMotor = new SparkMax(motorID, MotorType.kBrushless);

    // Get the closed loop controller object from the newly created motor controller object that is stored in
    // the rollerMotor variable.  We'll then create a refrence to that closed loop controller object in the
    // rollerCLC variable.
    rollerCLC = rollerMotor.getClosedLoopController();

    // Call the 'configure' method inside the motor controller object (stored in the rollerMotor
    // variable).  We'll then pass in the configuration parameters that the 'configure' method is looking for.
    rollerMotor.configure(Configs.CoralMechConfig.rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setVelocity(double velocity) {

    // Call the 'setRefrence' method inside the closed loop controller object
    // that we are refrencing in the rollerCLC variable.  We'll pass into the method
    // the velocity variable and the mode we want it to use.
    rollerCLC.setReference(velocity, ControlType.kVelocity);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

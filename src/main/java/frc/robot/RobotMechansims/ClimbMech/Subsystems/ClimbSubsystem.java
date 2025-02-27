// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.ClimbMech.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMechansims.MechanismConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  public final SparkMax climbMotorFront;

  public final SparkMax climbMotorBack;

  public final SparkClosedLoopController SparkCLC1;

  public final SparkClosedLoopController SparkCLC2;

  public ClimbSubsystem() { 

    climbMotorFront = new SparkMax(MechanismConstants.ClimberValues.motorFrontID, MotorType.kBrushless);

    climbMotorBack = new SparkMax(MechanismConstants.ClimberValues.motorBackID, MotorType.kBrushless);

    SparkCLC1 = climbMotorFront.getClosedLoopController();

    SparkCLC2 = climbMotorBack.getClosedLoopController();

  }

  public void goTP(double setPoint) {

    SparkCLC1.setReference(setPoint, ControlType.kPosition);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

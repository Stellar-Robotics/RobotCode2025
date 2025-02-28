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

  public final SparkMax climbMotor1;

  public final SparkMax climbMotor2;

  public final SparkClosedLoopController SparkCLC1;

  public final SparkClosedLoopController SparkCLC2;

  public ClimbSubsystem() { 

    climbMotor1 = new SparkMax(MechanismConstants.ClimberValues.motorID1, MotorType.kBrushless);

    climbMotor2 = new SparkMax(MechanismConstants.ClimberValues.motorID2, MotorType.kBrushless);

    SparkCLC1 = climbMotor1.getClosedLoopController();

    SparkCLC2 = climbMotor2.getClosedLoopController();

  }

  /** Sets the forward speed of the climber. Negative values will not change the direction of the motors. */
  public void setSpeed(double speed) {
    SparkCLC1.setReference(Math.abs(speed), ControlType.kVelocity);
    SparkCLC2.setReference(-Math.abs(speed), ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

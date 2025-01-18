// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.AlgaeIntake.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class AlgaeIntake extends SubsystemBase {

  private final SparkMax extensionMotor;
  private final SparkMax pickupMotor;  


  /** Creates a new AlgaeIntake. */
  public AlgaeIntake(int extensionMotorid, int pickupMotorid) {

    extensionMotor = new SparkMax(extensionMotorid, MotorType.kBrushless);
    pickupMotor = new SparkMax(pickupMotorid, MotorType.kBrushless);

    extensionMotor.configure(Configs.AlgaeIntakeConfig.extensionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pickupMotor.configure(Configs.AlgaeIntakeConfig.pickupMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // Speed was set to 0.5
  public void runPickup(double speed) {

    pickupMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}

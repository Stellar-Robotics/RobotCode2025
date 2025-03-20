// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.ClimbMech.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotUtilities.MiscUtils;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  public final SparkMax climbMotor1;

  public final SparkMax climbMotor2;

  public final SparkClosedLoopController SparkCLC1;

  public final SparkClosedLoopController SparkCLC2;

  public DoubleSolenoid lockSolenoid;

  private boolean lockState = false;

  public ClimbSubsystem(DoubleSolenoid lockSolenoid) { 

    climbMotor1 = new SparkMax(MechanismConstants.ClimberValues.motorID1, MotorType.kBrushless);

    climbMotor2 = new SparkMax(MechanismConstants.ClimberValues.motorID2, MotorType.kBrushless);

    SparkCLC1 = climbMotor1.getClosedLoopController();

    SparkCLC2 = climbMotor2.getClosedLoopController();

    climbMotor1.configure(Configs.ClimberConfig.MotorFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbMotor2.configure(Configs.ClimberConfig.MotorBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.lockSolenoid = lockSolenoid;

    // Lock solenoid is on by default
    this.lockSolenoid.set(Value.kForward);
  }

  /** Sets the forward speed of the climber. Negative values will not change the direction of the motors. */
  public void setPosition(double positionRotations) {
    SparkCLC1.setReference(MiscUtils.clamp(-80, 4, positionRotations), ControlType.kPosition);
  }

  public Command toggleLock(ClimbSubsystem subsystem, int direction) {
    return Commands.runOnce(() -> {
      // Retract
      if (direction == 1) {
        lockSolenoid.set(Value.kForward);
        lockState = false;
      // Extend
      } else if (direction == 2) {
        lockSolenoid.set(Value.kReverse);
        lockState = true;
      // Toggle
      } else {
        lockSolenoid.set(!lockState ? Value.kForward : Value.kReverse);
        lockState = !lockState;
      }
    }, subsystem);
  }

  // Move the climber, but only if the elevator clears it.
  public Command setClimber(boolean down) {
    if (RobotContainer.getSingletonInstance().getElevatorPosition() > 35) {
      return Commands.runOnce(() -> {
        this.setPosition(down ? 4 : -75);
      }, this);   
    } else {
      return Commands.runOnce(() -> {}, this);
    }
  }

  public Command resetClimber() {
    return this.toggleLock(this, 1) 
    .andThen(new WaitCommand(0.4))
    .andThen(
      Commands.runOnce(() -> {
        this.setPosition(0);
      }, this)
    );
  }

  public Command engageAndLock() {
    return new SequentialCommandGroup(
      this.setClimber(true),
      new WaitCommand(3),
      this.toggleLock(this, 2)
    );
  }

  public double getPostion() {
    return climbMotor1.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

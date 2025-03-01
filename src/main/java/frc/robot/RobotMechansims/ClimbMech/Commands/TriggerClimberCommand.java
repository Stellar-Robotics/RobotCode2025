// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.ClimbMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.ClimbMech.Subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TriggerClimberCommand extends Command {

  private ClimbSubsystem mechanism;

  /** Creates a new SetClimberPositionCommand. */
  public TriggerClimberCommand(ClimbSubsystem subsystem) {

    mechanism = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (MechanismConstants.ClimberValues.triggered) {
      return;
    } else {
      // Execute order 5413!
      mechanism.setSpeed(5);
      MechanismConstants.ClimberValues.triggered = true;
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

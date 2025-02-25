// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.CoralMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.CoralMech.Subsystems.CoralMech;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleCoralExtension extends Command {

  private CoralMech subsystem;

  /** Creates a new ToggleCoralExtension. */
  public ToggleCoralExtension(CoralMech mechanism) {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = mechanism;
    addRequirements(mechanism);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (MechanismConstants.CoralMechValues.lastPos) {
      subsystem.goToPosition(0);
      MechanismConstants.CoralMechValues.lastPos = false;
    } else {
      subsystem.goToPosition(44);
      MechanismConstants.CoralMechValues.lastPos = true;
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

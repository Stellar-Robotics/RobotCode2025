// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.CoralMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.CoralMech.Subsystems.CoralMech;

public class RunCoralMechCommand extends Command {

  // Create variables to hold our refrences to the constructor parameters.
  private final CoralMech subsystem;
  private final double velocity;

  
  public RunCoralMechCommand(CoralMech subsystem, double velocity) {
    
    // Set each of the variables we created above to refrence the parameters that are passed
    // into this class.
    this.subsystem = subsystem;
    this.velocity = velocity;

    // Tells the scheduler that this command will be controlling the CoralMech
    // object that was passed in.
    addRequirements(subsystem);

  }

  @Override // Not using for this class.
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Call the 'setVelocity' method inside the CoralMech subsystem, which
    // is refrenced in the 'subsystem' variable.  We'll pass in the velocity
    // variable to tell it how fast we want it to spin the motor.
    subsystem.setVelocity(velocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

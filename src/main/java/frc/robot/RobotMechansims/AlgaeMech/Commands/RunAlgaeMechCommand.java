package frc.robot.RobotMechansims.AlgaeMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.AlgaeMech.Subsystems.AlgaeMech;

public class RunAlgaeMechCommand extends Command {

    // declare the variables that will hold refrences to the passed
    // in AlgaeMech object and the speed.
    private AlgaeMech intake;
    private double speed;

    public RunAlgaeMechCommand(AlgaeMech intakeParam, double speedParam) {

        // Set the variables (we just declared) to hold a refrence to the
        // passed-in parameters from this constructor method. 
        this.intake = intakeParam;
        this.speed = speedParam;

        addRequirements(intakeParam);

    }

    @Override
    public void execute() {

        // Call the 'runPickup' method from the object stored in the
        // intake variable.  We'll pass in the variable that holds
        // the speed value that we want to run the motor at.
        intake.setSpeed(speed);
        
    }

    @Override
    public boolean isFinished() {
      return true;
    }
} 

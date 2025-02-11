package frc.robot.RobotMechansims.AlgaeMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.AlgaeMech.Subsystems.AlgaeMech;

public class RunAlgaeMechCommand extends Command {

    private AlgaeMech intake;
    private double speed;

    public RunAlgaeMechCommand(AlgaeMech intakeParam, double speedParam) {
        this.intake = intakeParam;
        this.speed = speedParam;
    }

    @Override
    public void execute() {
        this.intake.runPickup(speed);
    }
} 

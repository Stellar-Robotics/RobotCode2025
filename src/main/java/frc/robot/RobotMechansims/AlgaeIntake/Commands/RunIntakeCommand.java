package frc.robot.RobotMechansims.AlgaeIntake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.AlgaeIntake.Subsystems.AlgaeIntake;

public class RunIntakeCommand extends Command {

    private AlgaeIntake intake;
    private double speed;

    public RunIntakeCommand(AlgaeIntake intakeParam, double speedParam) {
        this.intake = intakeParam;
        this.speed = speedParam;
    }

    @Override
    public void execute() {
        this.intake.runPickup(speed);
    }
} 

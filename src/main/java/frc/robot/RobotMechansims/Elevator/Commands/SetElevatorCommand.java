package frc.robot.RobotMechansims.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator;

public class SetElevatorCommand extends Command {

    private Elevator elevator;
    private double position;

    public SetElevatorCommand(Elevator elev, double pos) {
        this.elevator = elev;
        this.position = pos;
    }

    @Override
    public void execute() {
        this.elevator.goToPositionClamped(position);
    }
} 

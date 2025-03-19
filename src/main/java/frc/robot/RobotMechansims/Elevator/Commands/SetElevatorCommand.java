package frc.robot.RobotMechansims.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator.POSITIONS;
import frc.robot.RobotMechansims.MechanismConstants.elevatorValues.ELEVATORPOSITION;

public class SetElevatorCommand extends Command {

    private Elevator elevator;
    private POSITIONS position;

    public SetElevatorCommand(Elevator elev, POSITIONS position) {
        this.elevator = elev;
        this.position = position;
        addRequirements(elevator);
    }

    @Override
    public void execute() {

        switch (position) {
            case LOW:
                this.elevator.goToPositionClamped(0);
                MechanismConstants.elevatorValues.currentPos = ELEVATORPOSITION.LOW;
                break;
            case ALGAELOW:
                this.elevator.goToPositionClamped(80);
                MechanismConstants.elevatorValues.currentPos = ELEVATORPOSITION.ALGAELOW;
                break;
            case MID:
                this.elevator.goToPositionClamped(65);
                MechanismConstants.elevatorValues.currentPos = ELEVATORPOSITION.MID;
                break; 
            case ALGAEHIGH:
                this.elevator.goToPositionClamped(163);
                MechanismConstants.elevatorValues.currentPos = ELEVATORPOSITION.ALGAEHIGH;
                break;
            case HIGH:
                this.elevator.goToPositionClamped(178);
                MechanismConstants.elevatorValues.currentPos = ELEVATORPOSITION.HIGH;
            default:
                break;
        }

        System.out.println("Elevator " + position);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
} 

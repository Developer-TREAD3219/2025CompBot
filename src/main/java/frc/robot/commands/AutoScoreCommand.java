package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.RumbleHelper;

public class AutoScoreCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final XboxController gunnerController;
    private final RumbleHelper rumbleHelper;

    public AutoScoreCommand(ElevatorSubsystem elevatorSubsystem, XboxController gunnerController) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.gunnerController = gunnerController;
        addRequirements(elevatorSubsystem);
        this.rumbleHelper = new RumbleHelper(gunnerController);
    }

    @Override
    public void initialize() {
        String commandStatus = "Waiting for command";
         


        // Add shift to the Left when the Left Trigger is pressed
        if (gunnerController.getLeftTriggerAxis() > 0.9) {
            if (gunnerController.getYButton()) {
                elevatorSubsystem.goToElevatorStow();
                commandStatus = "Success: Elevator can score L1 on the left";
            } else if (gunnerController.getXButton()) {
                elevatorSubsystem.goToElevatorL2();
                commandStatus = "Success: Elevator can score L2 on the left";
            } else if (gunnerController.getBButton()) {
                elevatorSubsystem.goToElevatorL3();
                commandStatus = "Success: Elevator can score L3 on the left";
            } else if (gunnerController.getAButton()) {
                elevatorSubsystem.goToElevatorL4();
                commandStatus = "Success: Elevator can score L4 on the left";
            }
        }

        // Add shift to the Right when the Right Trigger is pressed
        if (gunnerController.getRightTriggerAxis() > 0.9) {
            if (gunnerController.getAButton()) {
                //TODO: Example Rumble Implementation
                rumbleHelper.rumbleForDuration(0.3, 2, 0.09); // Rumble for 0.3 seconds twice at low intensity
                elevatorSubsystem.goToElevatorStow();
                commandStatus = "Success: Elevator can score L1 on the right";
            } else if (gunnerController.getYButton()) {
                elevatorSubsystem.goToElevatorL2();
                commandStatus = "Success: Elevator can score L2 on the right";
            } else if (gunnerController.getXButton()) {
                elevatorSubsystem.goToElevatorL3();
                commandStatus = "Success: Elevator can score L3 on the right";
            } else if (gunnerController.getBButton()) {
                elevatorSubsystem.goToElevatorL4();
                commandStatus = "Success: Elevator can score L4 on the right";
                
            }
        }

        // Print the latest command status
        System.out.println(commandStatus);
    }

    @Override
    public void execute() {
        // Implementation of execute method
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
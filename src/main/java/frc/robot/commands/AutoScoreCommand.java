package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoScoreCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;
    XboxController gunnerController;

public AutoScoreCommand (ElevatorSubsystem elevatorSubsystem, XboxController gunnerController) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.gunnerController = gunnerController;
        addRequirements(elevatorSubsystem);
    }

//LT= Score Left Coral 
//RT= Score Right Coral
//X= Level 1 for Coral Auto (should automatically got to selected Level)
//Y= Level 2 for Coral Auto (should automatically got to selected Level)
//B= Level 3 for Coral Auto (should automatically got to selected Level)
//A= Level 4 for Coral Auto (should automatically got to selected Level)
//DPad Up= Going Up to Selected Level and should be Combined with Level Auto
//DPad Down= Going Down to Selected Level and should be Combined with Level Auto

    // m_gunnerController.getAButton() ||
    //         m_gunnerController.getYButton() ||
    //         m_gunnerController.getXButton() ||
    //         m_gunnerController.getBButton() ) &&
    //        (m_gunnerController.getLeftTriggerAxis() > 0.9 ||
    //         m_gunnerController.getRightTriggerAxis() > 0.9);
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        //TODO Add shift to the Left when the Left Trigger is pressed
        if (gunnerController.getLeftTriggerAxis() > 0.9) {
            if (gunnerController.getYButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL1);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L1");      
            }
            else if (gunnerController.getXButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL2);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L2");      
            }
            else if (gunnerController.getBButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL3);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L3");      
            }
            else if (gunnerController.getAButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL4);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L4"); 
            }
        }
        //TODO Add shift to the Right when the Right Trigger is pressed
        if (gunnerController.getRightTriggerAxis() > 0.9) {
            if (gunnerController.getAButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL1);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L1");      
            }
            else if (gunnerController.getYButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL2);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L2");      
            }
            else if (gunnerController.getXButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL3);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L3");      
            }
            else if (gunnerController.getBButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL4);
                SmartDashboard.putString("elevatorTest", "Success: Elevator can score L4"); 
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotors();
    }

    // TODO: DO WE NEED THIS?
    // Returns true when the command should end.
    // @Override
    // public boolean isFinished() {
    //     // return timer.hasElapsed(TimerConstants.kTimeToIndexGamepiece);
    //     return elevatorSubsystem.hasGamepiece();
    //     // return false;
    // }
}

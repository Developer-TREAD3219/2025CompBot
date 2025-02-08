package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class BeginEndMatch extends Command {

    /**
     * 1. Lower elevator 
     * 2. Open the intake door (we may not be using a servo to open the intake door)
     * 3. Rotate climber clockwise 270 degrees
     * Drivers will maneuver into position
     * 4. roll climber into climb position
     */
    ElevatorSubsystem elevatorSubsystem;
    // IntakeSubsystem intakeSubsystem; // we may not be using a servo to open the intake door
    ClimberSubsystem climberSubsystem;

    public BeginEndMatch (ElevatorSubsystem elevatorSubsystem, ClimberSubsystem climberSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        // this.intakeSubsystem = intakeSubsystem;  // we may not be using a servo to open the intake door
        this.climberSubsystem = climberSubsystem; 
       
        addRequirements(elevatorSubsystem);
        // addRequirements(intakeSubsystem); // we may not be using a servo to open the intake door
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        //TODO: ENABLE CLIMBER SUBSYSTEM.  why??? if this is the only place it's called?
        elevatorSubsystem.homeElevator();
        // intakeSubsystem.openServo();  // we may not be using a servo to open the intake door
        climberSubsystem.rotateClimber(270);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

}
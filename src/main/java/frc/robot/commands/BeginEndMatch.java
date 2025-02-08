package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class BeginEndMatch extends Command {

    /**
     * 1. Lower elevator 
     * 2. Open the intake door 
     * 3. Rotate climber out 90 degrees
     * Drivers will maneuver into position
     * 4. roll climber into climb position
     */
    ElevatorSubsystem elevatorSubsystem;
    IntakeSubsystem intakeSubsystem;
    ClimberSubsystem climberSubsystem;

    public BeginEndMatch (ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.climberSubsystem = climberSubsystem; 
       
        addRequirements(elevatorSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.homeElevator();
        intakeSubsystem.openServo();
        climberSubsystem.rotateClimber(90);
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

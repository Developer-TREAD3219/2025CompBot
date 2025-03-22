package frc.robot.commands;

// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants.coralDeliveryConstants;

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
    // Servo intakeServo;
    Talon hatchMotor;
    Timer pullTimer;

    public BeginEndMatch (ElevatorSubsystem elevatorSubsystem, ClimberSubsystem climberSubsystem, Talon hatchMotor) {
        this.elevatorSubsystem = elevatorSubsystem;
        // this.intakeServo = intakeServo;
        this.hatchMotor = hatchMotor;
        this.climberSubsystem = climberSubsystem;
        this.pullTimer = new Timer();
       
        addRequirements(elevatorSubsystem);
        // addRequirements(intakeSubsystem); // we may not be using a servo to open the intake door
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        //TODO: ENABLE CLIMBER SUBSYSTEM.  why??? if this is the only place it's called? Only if we need it
        // elevatorSubsystem.homeElevator();
        // intakeServo.set(coralDeliveryConstants.kIntakeServoOpen);
        hatchMotor.set(1);
        // climberSubsystem.rotateClimber(270);
        pullTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (pullTimer.get()>3.0){
        //     isFinished();
        // }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("a;lsdkfghaoiwg");
         hatchMotor.stopMotor();
    }
}
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    
    /**
     This command lowers the elevator to score L1.
    */
    public class ElevatorL1 extends Command {
      // The subsystem the command runs on
      ElevatorSubsystem subsystem;
    
      public ElevatorL1(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
      }
    
      @Override
      public void initialize() {
        subsystem.setPositionInches(ElevatorConstants.kL1);
        SmartDashboard.putString("elevatorTest", "Success: Elevator can score L1");  
      }
      
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
      }

    // we shouldn't need an isFinished() method if our call is in the initialize and not the execute.
    //   // If this returns true, the command will end.
    //   @Override
    //   public boolean isFinished() {
    //     return subsystem.isAtPosition(ElevatorPosition.POSITION_1);
    //   }
      
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        subsystem.stopMotors();
      }
    }


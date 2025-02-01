package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    
    /**
     This command raises the elevator to score L1 after
     */
    public class ElevatorL1 extends Command {
      // The subsystem the command runs on

    
      public ElevatorL1() {
        //m_hatchSubsystem = subsystem;
        //addRequirements(m_hatchSubsystem);
         SmartDashboard.putString("elevatorTest", "Success: Elevator can score L1");         

      }
    
      @Override
      public void initialize() {
      //  m_hatchSubsystem.grabHatch();

      }
    
      @Override
      public boolean isFinished() {
        return true;
      }
    }


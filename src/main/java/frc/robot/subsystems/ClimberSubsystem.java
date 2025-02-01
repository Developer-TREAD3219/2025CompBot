//TODO: We should add some sort of toggle to this system. We don't want to move the arm when we are ready to park
// We can accomplish by adding an isEnabled bool that starts as false and gets set as true when we enter climb mode
// Extend and retract should first check if isEnabled before they run the motor
// 

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
    public class ClimberSubsystem extends SubsystemBase {
            public ClimberSubsystem() {
                
                    
                


            }

         public void Extend() {
                    // code for extending Climber
                    System.out.println("");


                }

    
                public void retract(){
                    // code for retracting Climber

        
                }
                public void periodic() {
                    // This method will be called once per scheduler run 

         };
}

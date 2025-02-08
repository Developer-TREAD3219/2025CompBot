//TODO: We should add some sort of toggle to this system. We don't want to move the arm when we are ready to park
// We can accomplish by adding an isEnabled bool that starts as false and gets set as true when we enter climb mode
// Extend and retract should first check if isEnabled before they run the motor
// 

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
    public class ClimberSubsystem extends SubsystemBase {


            private boolean isEnabled;

            public ClimberSubsystem() {
                
            //make private boolean to make sure only the toggle will cause it to start
            //create a way to disable and enable the extend       
                

public class ClimberSubsystem extends SubsystemBase {

    //private final SparkMax climberMotor;

    // public ClimberSubsystem() {
        
            
         public void Extend() {
            if (isEnabled) {
                
            
                    // code for extending Climber
                    System.out.println("Extending Arm");
    // }

                }
            }
    
                public void retract(){
                    if (isEnabled){
                    System.out.println("Retracting Arm...");
                    // code for retracting Climber

                    }
                }
                public void periodic() {
                    // This method will be called once per scheduler run 

         };

         public void toggleEnabled() {
                isEnabled = !isEnabled;
         }

         public void enable() {
                isEnabled = true;
         }

         public void disable(){

                isEnabled = false;
         }
        }


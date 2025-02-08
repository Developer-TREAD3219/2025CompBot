//TODO: We should add some sort of toggle to this system. We don't want to move the arm when we are ready to park
// We can accomplish by adding an isEnabled bool that starts as false and gets set as true when we enter climb mode
// Extend and retract should first check if isEnabled before they run the motor
// 

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor;
    private boolean isEnabled = false;

            //make private boolean to make sure only the toggle will cause it to start
            //create a way to disable and enable the extend       

    public ClimberSubsystem() {
        climberMotor = new SparkMax(ClimberConstants.KClimberMotorID, MotorType.kBrushless); 
    }
    
    public void rotateClimber(int degrees) {
        // code for rotating Climber
                isEnabled = false;
    }
 
    public void isClimber(int degrees) {
        // code for rotating Climber
                isEnabled = false;
    }
}
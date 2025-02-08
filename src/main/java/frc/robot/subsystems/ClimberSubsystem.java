//TODO: We should add some sort of toggle to this system. We don't want to move the arm when we are ready to park
// We can accomplish by adding an isEnabled bool that starts as false and gets set as true when we enter climb mode
// Extend and retract should first check if isEnabled before they run the motor
// 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    SparkFlex climberMotor = new SparkFlex(ClimberConstants.KClimberMotorID, MotorType.kBrushless);
    RelativeEncoder climberEncoder = climberMotor.getEncoder();
    //SparkPIDController pitchPidController;

    private boolean isEnabled = false;

            //make private boolean to make sure only the toggle will cause it to start
            //create a way to disable and enable the extend       

    public ClimberSubsystem() {
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
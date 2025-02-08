//TODO: We should add some sort of toggle to this system. We don't want to move the arm when we are ready to park
// We can accomplish by adding an isEnabled bool that starts as false and gets set as true when we enter climb mode
// Extend and retract should first check if isEnabled before they run the motor
// 

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(ClimberConstants.KClimberMotorID, MotorType.kBrushless);  
    }
    
    public void rotateClimber(int degrees) {
        // code for rotating Climber


    }
}

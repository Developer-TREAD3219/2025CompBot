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

        // set the motor's target velocity to a value proportional to the desired rotation angle "x" using your
        // programming language and the SparkFlex controller's CAN interface, taking into account the motor's
        // gearing and the desired rotation direction (clockwise or counter clockwise)

        // set neo vortex motor to 270 degrees using spark flex controller
        // climberEncoder.getPosition();
        // climberEncoder.setPosition(270);

                // Assuming the motor controller uses degrees for position control
                //climberMotor.set(ControlMode.Position, 270.0);

        climberMotor.set(ClimberConstants.kClimberSpeed);
    }
 
    // public void isClimber(int degrees) {
    //     // code for rotating Climber
    //             isEnabled = false;
    // }

    public void stopClimber() {
        climberMotor.set(0);
    }
}
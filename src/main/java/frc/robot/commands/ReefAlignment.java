package frc.robot.commands;

import frc.robot.subsystems.LimeLightSubsystem;

import java.lang.reflect.Array;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ReefAlignment extends Command {
    LimeLightSubsystem m_LimeLightSubsystem;
    DriveSubsystem m_DriveSubsystem;
    ElevatorSubsystem m_ElevatorSubsystem;
    boolean alignedStage1;
    boolean isLeftAligning;
    double kRight;
    private int targetSeenCounter = 0;
    //anti jitter logic 2oo
    private final SlewRateLimiter xLimiter   = new SlewRateLimiter(0.5);
    private final SlewRateLimiter yLimiter   = new SlewRateLimiter(0.5);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);

    public ReefAlignment(LimeLightSubsystem limelight, DriveSubsystem drive, ElevatorSubsystem elevator, boolean isLeft) {
        m_LimeLightSubsystem = limelight;
        m_DriveSubsystem = drive;
        m_ElevatorSubsystem = elevator;
        isLeftAligning = isLeft;
        // Left right alignment chooser

        addRequirements(m_LimeLightSubsystem, m_DriveSubsystem);

    }

    @Override
    public void initialize(){
        alignedStage1 = false;
    }

    @Override
    public void execute(){
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        double xPosition = botpose[0];
        double yPosition = botpose[2];
        double yaw = botpose[4];
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double rotAdjust = 0;
        double yAdjust = 0;
        double xAdjust = 0;

        System.out.println("X  : " + xPosition);
        System.out.println("Y  : " + yPosition);
        System.out.println("Yaw: " + yaw);

        //Potential anti jitter fix if the issue is dropping tv for a moment
        if (tv >= 1.0) {
            targetSeenCounter = 5; // reset the counter any time we see the target
        } else if (targetSeenCounter > 0) {
            targetSeenCounter--;
        }
    
        boolean hasTarget = (targetSeenCounter > 0);


        if (yaw > 2.5){
            rotAdjust = -0.05;
        }
        if (yaw < -2.5){
            rotAdjust = 0.05;
        }
        // left/right allignment
        if (isLeftAligning){ //left alignment these need tuning
            if (xPosition > .191){
                yAdjust = 0.2;
                if (Math.abs(xPosition-.19) < .1){
                    yAdjust /= 5;
                }
            }
            else if (xPosition < .189){
                yAdjust = -0.2;
                if (Math.abs(xPosition-.19) < .1){
                    yAdjust /= 5;
                }
            }
        }
        else{// right alignment. In theory these are correct
            if (xPosition > -.179){
                yAdjust = 0.2;
                if (Math.abs(xPosition+.18) < .1){
                    yAdjust /= 5;
                }
            }
            else if (xPosition < -.181){
                yAdjust = -0.2;
                if (Math.abs(xPosition+.18) < .1){
                    yAdjust /= 5;
                }
            }
        }

        if (yPosition > .7){
            xAdjust = -0.05;
        }
        if (yPosition < .6){
            xAdjust = 0.05;
        }

        //slew rate proccessing to smooth things out
        double xCmd   = xLimiter.calculate(xAdjust);
        double yCmd   = yLimiter.calculate(yAdjust);
        double rotCmd = rotLimiter.calculate(rotAdjust);        

        // drive to pose code
        if (!alignedStage1 && hasTarget && Math.abs(xAdjust)+Math.abs(yAdjust)+Math.abs(rotAdjust) != 0){
        m_DriveSubsystem.drive(xCmd, yCmd, rotCmd, false);
        }
        if (!alignedStage1 && Math.abs(xAdjust)+Math.abs(yAdjust)+Math.abs(rotAdjust) ==0){
            alignedStage1 = true;
        }
        if (alignedStage1){
            m_DriveSubsystem.drive(-0.1, 0, 0, false);
        }
    }
}


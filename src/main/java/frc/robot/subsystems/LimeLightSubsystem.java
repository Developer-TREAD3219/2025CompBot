package frc.robot.subsystems;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;

//import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Pigeon2 m_gyro; // Add this line to declare m_gyro
    // Limelight for reading AprilTags
    private final String limelightCam = VisionConstants.kCameraName;
    private LimelightHelpers.LimelightResults result;
    private LimelightHelpers.LimelightTarget_Fiducial currentLock;
    private double currentLockDistance = Double.MAX_VALUE;

    // List of Reef Tags
    private final int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    
    // Target lock buffer duration: how long we're ok with keeping an old result
    private final double bufferTime = 1.0;
    private double lastUpdateTime = 0.0;

    // Target lock on maxDistance: the farthest away in meters that we want to lock on from
    private final double maxLockOnDistance = 0.5;

    public LimeLightSubsystem(DriveSubsystem driveSubsystem) {
        // Initialize Limelight settings if needed
        this.m_poseEstimator = driveSubsystem.getPoseEstimator();
        this.m_gyro = driveSubsystem.getGyro(); // Initialize with appropriate parameters
    }

    public void driverMode() {
        LimelightHelpers.setLEDMode_ForceOff(limelightCam);
        LimelightHelpers.setStreamMode_Standard(limelightCam);
    }

    public void detectAprilTags() {
        LimelightHelpers.setLEDMode_ForceOn(limelightCam);
        LimelightHelpers.setPipelineIndex(limelightCam, 0);
    }

   
    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        result = LimelightHelpers.getLatestResults(limelightCam);

        // System.out.println(" result = " + result.targets_Fiducials.length);
        // System.out.println(" Yaw=" + getYaw());
        // System.out.println(" Skew=" + getSkew());
        // System.out.println(" Pitch=" + getPitch());

        // Clear our lock on if it's been too long
        if (currentLock != null && (currentTime - lastUpdateTime > bufferTime)) {
            currentLock = null;
            currentLockDistance = Double.MAX_VALUE;
        }

        // Look for the last result that has a valid target
        if (result != null && result.targets_Fiducials.length > 0) {
            for (LimelightHelpers.LimelightTarget_Fiducial target : result.targets_Fiducials) {
                if (Arrays.stream(reefTags).anyMatch(id -> id == target.fiducialID)) {
                    currentLock = target;
                    lastUpdateTime = currentTime;
                    break;
                }
            }
            System.out.println("target found");
        }
    }

    // called from Drive subsystem
    // 
    public void updateRobotOrientation() {
        LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        boolean doRejectUpdate = false;
        if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
    }

    public void attemptReefLockon() {
        System.out.println("fiducials length=" + result.targets_Fiducials.length);
        if (result != null && result.targets_Fiducials.length > 0) {
            for (LimelightHelpers.LimelightTarget_Fiducial target : result.targets_Fiducials) {
                if (Arrays.stream(reefTags).anyMatch(id -> id == target.fiducialID)) {
                    double distance = get2dDistance(target);
                    if (distance < maxLockOnDistance && distance < currentLockDistance) {
                        currentLock = target;
                        currentLockDistance = distance;
                    }
                }
            }
        }
    }

    public int getApriltagID() {
        return currentLock != null ? (int) currentLock.fiducialID : -1;
    }

    public double getSkew() {
        return currentLock != null ? currentLock.ts : 0.0;
    }

    public double getYaw() {
        return currentLock != null ? currentLock.tx : 0.0;
    }

    public double getPitch() {
        return currentLock != null ? currentLock.ty : 0.0;
    }

    // public boolean isAmbiguousPose() {
    //     return currentLock != null && currentLock.ambiguity > 0.1;
    // }

    public double get2dDistance(LimelightTarget_Fiducial target) {
        return target.getTargetPose_RobotSpace().getTranslation().toTranslation2d().getNorm();
    }

    @Override
    public void periodic() {
        System.out.println("id=" + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0));
        update();
        if (currentLock != null) {
            // SmartDashboard.putNumber("Apriltag ID", getApriltagID());
            // SmartDashboard.putNumber("Skew", getSkew());
            // SmartDashboard.putNumber("Yaw", getYaw());
            // SmartDashboard.putNumber("Pitch", getPitch());
            // // SmartDashboard.putBoolean("Ambiguous Pose", isAmbiguousPose());
            // SmartDashboard.putNumber("Distance", get2dDistance(currentLock));
        }
        // This method will be called once per scheduler run

        // tv int 1 if valid target exists. 0 if no valid targets exist
        //tx double	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
        //ty double	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
        // double tv = NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("tv").getDouble(0);
        // double tx = NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("tx").getDouble(0);
        // double ty = NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("ty").getDouble(0);
        // double ta = NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("ta").getDouble(0);
        
    }

   

    //NetworkTableInstance.getDefault().getTable("limeLightCam").getEntry("<variablename>").getDouble(0);
}
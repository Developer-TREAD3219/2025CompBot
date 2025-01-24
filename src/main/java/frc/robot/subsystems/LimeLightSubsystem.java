package frc.robot.subsystems;

// Currently not used
//import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import java.util.List;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
// Currently not used
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class LimeLightSubsystem extends SubsystemBase {
    //limelight for reading Apriltags 
    PhotonCamera camera;
    public PhotonPipelineResult result;
    public PhotonTrackedTarget bestTarget;

    public LimeLightSubsystem() {
        camera = new PhotonCamera(VisionConstants.kCameraName);
    }

    public void driverMode() {
        camera.setDriverMode(true);
        camera.setLED(VisionLEDMode.kOff);
    }

    public void detectAprilTags(){
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);
        camera.setLED(VisionLEDMode.kOff);
    }

    public void update() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            result = results.get(0);
        }
    }
    public void findBestTarget(){
        bestTarget = result.getBestTarget();
    }
    public int getApriltagID(){
        return bestTarget.getFiducialId();
    }
    public double getSkew(){
        return bestTarget.getSkew();
    }
    public double getYaw(){
        return bestTarget.getYaw();
    }
    public double getPitch(){
        return bestTarget.getPitch();
    }
    public boolean isAmbiguousPose(){
        return bestTarget.getPoseAmbiguity() > 0.1;
    }
    public Transform3d getBestCameraToTarget(){
        return bestTarget.getBestCameraToTarget();
    }
    @Override
    public void periodic(){
        update();
        if (result.hasTargets()){
            findBestTarget();
            SmartDashboard.putNumber("Apriltag ID", getApriltagID());
            SmartDashboard.putNumber("Skew", getSkew());
            SmartDashboard.putNumber("Yaw", getYaw());
            SmartDashboard.putNumber("Pitch", getPitch());
            SmartDashboard.putBoolean("Ambiguous Pose", isAmbiguousPose());
        }
        //This method will be called once per scheduler run
    }
}

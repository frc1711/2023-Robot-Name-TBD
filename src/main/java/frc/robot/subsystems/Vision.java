package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import claw.subsystems.SubsystemCLAW;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision extends SubsystemCLAW{
    
    private static Vision visionInstance;

    public static Vision getInstance() {
        if (visionInstance == null) visionInstance = new Vision();
        return visionInstance;
    }

    //TODO: Properly implement Limelight
    private VideoCapture usbCamera;
    private final Mat mat;
    private AprilTagDetector detector;
    private AprilTagPoseEstimator estimator;

    private Vision () {
        usbCamera = new VideoCapture();
        mat = new Mat();
    }

    //Use Mat images from the USB camera to scan for AprilTags
    public AprilTagDetection detectAprilTags () {
        AprilTagDetection[] detectionArray;
        if (usbCamera.retrieve(mat) == true) {
            detectionArray = detector.detect(mat);
            return detectionArray[0];
        }
        else return null;
    }
    
    //Return Pose3d of any AprilTags provided
    public Transform3d estimateTag (AprilTagDetection tag) {
        return estimator.estimate(tag);
    }

    //Return Translation3d of any AprilTags provided
    public Transform3d getTagDistance (AprilTagDetection detection) {
        return estimator.estimate(detection);
    }

  @Override
  public void periodic() {}
}
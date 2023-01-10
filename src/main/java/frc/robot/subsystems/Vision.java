package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import claw.subsystems.SubsystemCLAW;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemCLAW{
    
    private static Vision visionInstance;

    public static Vision getInstance() {
        if (visionInstance == null) visionInstance = new Vision();
        return visionInstance;
    }

    //TODO: Properly implement Limelight
    private final NetworkTableInstance limelight;
    private VideoCapture usbCamera;
    private Mat mat;
    private AprilTagDetector detector;
    private AprilTagPoseEstimator estimator;

    private Vision () {
        usbCamera = new VideoCapture();
        mat = new Mat();
        limelight = NetworkTableInstance.getDefault();

    }

    public AprilTagDetection detectAprilTags () {
        AprilTagDetection[] detectionArray;
        if (usbCamera.retrieve(mat) == true) {
            detectionArray = detector.detect(mat);
            return detectionArray[0];
        }
        else return null;
    }

    public AprilTagPoseEstimate estimateTag () {
        return estimator.estimateOrthogonalIteration(detectAprilTags(), 1);
    }

  @Override
  public void periodic() {
    if (detectAprilTags() != null) estimateTag();
  }
}

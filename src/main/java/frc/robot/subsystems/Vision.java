package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import claw.subsystems.SubsystemCLAW;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemCLAW{
    
    private static Vision visionInstance;

    public static Vision getInstance() {
        if (visionInstance == null) visionInstance = new Vision();
        return visionInstance;
    }

    //TODO: Properly implement Limelight
    private final NetworkTable limelight;
    private VideoCapture usbCamera;
    private Mat mat;
    private AprilTagDetector detector;
    private AprilTagPoseEstimator estimator;
    private NetworkTableEntry 
            tv,
            tx,
            ty,
            ta;

    private Vision () {
        usbCamera = new VideoCapture();
        mat = new Mat();
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        limelight.getEntry("ledMode").setDouble(0); //Sets the LED to desired status. 0 to follow current pipeline, 1 for off, 2 for blinking, 3 for on
        limelight.getEntry("camMode").setDouble(0); //Enables or disables vision processing. 0 for processing, 1 for drive camera
        limelight.getEntry("pipeline").setDouble(0) //Changes the pipeline of the Limelight
    }

    

    //Checks if Limelight sees a target, returns false if the value is zero, true if it is 1
    public boolean seesTarget () {
        return tv.getDouble(0) == 1;
    }

    //Returns the horizontal offset of the camera to the target
    public double getHorizontalOffset () {
        return tx.getDouble(0);
    }

    //Returns the vertical offset of the camera to the target
    public double getVerticalOffset () {
        return ty.getDouble(0);
    }

    //Use Mat images from the USB camera to scan for AprilTags and store their locations
    public AprilTagDetection detectAprilTags () {
        AprilTagDetection[] detectionArray;
        if (usbCamera.read(mat) == true) {
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

    public boolean isStart () {
        return usbCamera.isOpened();
    }

    public void start () {
        usbCamera.open(0);
    }

    public void stop () {
        usbCamera.release();
    }

  @Override
  public void periodic() {}
}
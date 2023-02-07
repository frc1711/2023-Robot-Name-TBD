package frc.robot.limelight;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    
    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");
    private static final long SNAPSHOT_RESET_MILLIS = 1000;
    
    /**
     * Basic targeting data
     */
    private static final NetworkTableEntry
        ENTRY_TV =      TABLE.getEntry("tv"),                   // Whether the limelight has any valid targets (0 or 1)
        ENTRY_TX =      TABLE.getEntry("tx"),                   // Horizontal offset from crosshair to target
        ENTRY_TY =      TABLE.getEntry("ty"),                   // Vertical offset from crosshair to target
        ENTRY_TA =      TABLE.getEntry("ta"),                   // Target area of image (percentage)
        ENTRY_TL =      TABLE.getEntry("tl"),                   // Pipeline's latency contribution (ms)
        ENTRY_TSHORT =  TABLE.getEntry("tshort"),               // Sidelength of shortest side of the fitted bounding box (pixels)
        ENTRY_TLONG =   TABLE.getEntry("tlong"),                // Sidelength of longest side of the fitted bounding box (pixels)
        ENTRY_THOR =    TABLE.getEntry("thor"),                 // Horizontal sidelength of the rough bounding box (pixels)
        ENTRY_TVERT =   TABLE.getEntry("tvert"),                // Vertical sidelength of the rough bounding box (pixels)
        ENTRY_GETPIPE = TABLE.getEntry("getpipe"),              // True active pipeline index of the camera (0 .. 9)
        ENTRY_JSON =    TABLE.getEntry("json"),                 // Full JSON dump of targeting results
        ENTRY_TCLASS =  TABLE.getEntry("tclass"),               // Class ID of primary neural detector result or neural classifier result
        ENTRY_TC =      TABLE.getEntry("tc"),                   // Get the average HSV color underneath the crosshair region as a NumberArray
        ENTRY_TPOSE =   TABLE.getEntry("targetpose_robotspace"),// Get the pose of the target relative to the robot in an array of 6 ints
        ENTRY_TID =     TABLE.getEntry("tid");                  // Get the ID of the tag identified (Only effective for AprilTags)
    
    // Basic target recognition
    
    public static boolean hasValidTarget () {
        return ENTRY_TV.getInteger(0) == 1;
    }
    
    public static Optional<TargetData> getTarget () {
        if (!hasValidTarget()) {
            return Optional.empty();
        } else {
            return Optional.of(new TargetData(
                ENTRY_TX.getDouble(0),
                ENTRY_TY.getDouble(0),
                ENTRY_TA.getDouble(0),
                ENTRY_TSHORT.getDouble(0),
                ENTRY_TLONG.getDouble(0),
                ENTRY_THOR.getDouble(0),
                ENTRY_TVERT.getDouble(0),
                (int)ENTRY_TCLASS.getInteger(-1),
                ENTRY_TC.getDoubleArray(new double[0])
            ));
        }
    }

    public static boolean hasAprilTag() {
        return ENTRY_TID.getDouble(0) != 0;
    }
    public static Optional<AprilTagData> getAprilTag () {

        if (hasAprilTag()) {
            return Optional.of(new AprilTagData(
                ENTRY_TID.getDouble(0), 
                ENTRY_TX.getDouble(0), 
                ENTRY_TY.getDouble(0), 
                ENTRY_TPOSE.getDoubleArray(new double[0])
                ));
        } else {
            return Optional.empty();
        }
    }
    
    public static record TargetData (
        double horizontalOffset,
        double verticalOffset,
        double targetArea,
        double shortSideLength,
        double longSideLength,
        double boundingBoxWidth,
        double boundingBoxHeight,
        int classId,
        double[] crosshairHSV
    ) { }
    
    public static record AprilTagData (
        double targetID,
        double verticalOffset,
        double horizontalOffset,
        double[] targetPose
    ) { }
    // Misc. data
    
    public static double getPipelineLatency () {
        return ENTRY_TL.getDouble(0);
    }
    
    public static String getJSONDump () {
        return ENTRY_JSON.getString("{}");
    }

    public static int getTargetID () {
        return (int)ENTRY_TID.getInteger(0);
    }
    
    public static int getActivePipeline () {
        return (int)ENTRY_GETPIPE.getInteger(-1);
    }
    
    // Camera controls
    
    private static final NetworkTableEntry
        ENTRY_LED_MODE          = TABLE.getEntry("ledMode"),
        ENTRY_CAMERA_MODE       = TABLE.getEntry("camMode"),
        ENTRY_PIPELINE_SELECT   = TABLE.getEntry("pipeline"),
        ENTRY_STREAM_MODE       = TABLE.getEntry("stream"),
        ENTRY_SNAPSHOT          = TABLE.getEntry("snapshot"),
        ENTRY_CROP_RECTANGLE    = TABLE.getEntry("crop");
    
    public static void setLEDMode (LEDMode mode) {
        ENTRY_LED_MODE.setDouble(mode.mode);
    }
    
    public static enum LEDMode {
        PIPELINE_DEFAULT    (0),
        FORCE_OFF           (1),
        FORCE_BLINK         (2),
        FORCE_ON            (3);
        
        private int mode;
        private LEDMode (int mode) {
            this.mode = mode;
        }
        
    }
    
    public static void setCameraMode (CameraMode mode) {
        ENTRY_CAMERA_MODE.setDouble(mode.mode);
    }
    
    public static enum CameraMode {
        VISION_PROCESSOR    (0),
        DRIVER_CAMERA       (1);
        
        private int mode;
        private CameraMode (int mode) {
            this.mode = mode;
        }
        
    }
    
    public static void setPipeline (int pipelineIndex) {
        ENTRY_PIPELINE_SELECT.setDouble(pipelineIndex);
    }
    
    public static void setStreamMode (StreamMode mode) {
        ENTRY_STREAM_MODE.setDouble(mode.mode);
    }
    
    public static enum StreamMode {
        STANDARD        (0),
        PiP_MAIN        (1),
        PiP_SECONDARY   (2);
        
        private int mode;
        private StreamMode (int mode) {
            this.mode = mode;
        }
        
    }
    
    private static long lastSnapshotActionTime = 0;
    
    private static boolean canTakeSnapshotAction () {
        return lastSnapshotActionTime + SNAPSHOT_RESET_MILLIS < System.currentTimeMillis();
    }
    
    private static boolean canResetSnapshot () {
        return canTakeSnapshotAction() && ENTRY_SNAPSHOT.getDouble(0) == 1;
    }
    
    public static boolean canTakeSnapshot () {
        return canTakeSnapshotAction() && ENTRY_SNAPSHOT.getDouble(1) == 0;
    }
    
    public static void takeSnapshot () {
        if (canTakeSnapshot()) {
            lastSnapshotActionTime = System.currentTimeMillis();
            ENTRY_SNAPSHOT.setDouble(1);
        }
    }
    
    private static void resetSnapshot () {
        if (canResetSnapshot()) {
            lastSnapshotActionTime = System.currentTimeMillis();
            ENTRY_SNAPSHOT.setDouble(0);
        }
    }
    
    public static void setCropRectangle (double minX, double minY, double maxX, double maxY) {
        ENTRY_CROP_RECTANGLE.setDoubleArray(new double[]{ minX, minY, maxX, maxY });
    }
    
    public static void update () {
        if (canResetSnapshot())
            resetSnapshot();
    }
    
}

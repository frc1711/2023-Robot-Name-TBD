package frc.robot.limelight;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Limelight {

    private static final ArrayList<Limelight> allLimelights = new ArrayList<>();

    public static final Limelight
        INTAKE_LIMELIGHT = new Limelight("intake", "10.17.11.17"),
        ARM_LIMELIGHT = new Limelight("arm", "10.17.11.16"); 

    private NetworkTable TABLE;
    private static final long SNAPSHOT_RESET_MILLIS = 1000;
    private MjpegServer cameraServer;
    
    /**
     * Basic targeting data
     */
    private final NetworkTableEntry
        ENTRY_TV,                   // Whether the limelight has any valid targets (0 or 1)
        ENTRY_TX,                   // Horizontal offset from crosshair to target
        ENTRY_TY,                   // Vertical offset from crosshair to target
        ENTRY_TA,                   // Target area of image (percentage)
        ENTRY_TL,                   // Pipeline's latency contribution (ms)
        ENTRY_TSHORT,               // Sidelength of shortest side of the fitted bounding box (pixels)
        ENTRY_TLONG,                // Sidelength of longest side of the fitted bounding box (pixels)
        ENTRY_THOR,                 // Horizontal sidelength of the rough bounding box (pixels)
        ENTRY_TVERT,                // Vertical sidelength of the rough bounding box (pixels)
        ENTRY_GETPIPE,              // True active pipeline index of the camera (0 .. 9)
        ENTRY_JSON,                 // Full JSON dump of targeting results
        ENTRY_TCLASS,               // Class ID of primary neural detector result or neural classifier result
        ENTRY_TC,                   // Get the average HSV color underneath the crosshair region as a NumberArray
        ENTRY_TPOSE,                // Get the pose of the target relative to the robot in an array of 6 doubles
        ENTRY_RPOSE,                // The robot pose in fieldspace
        ENTRY_TID;                  // Get the ID of the tag identified (Only effective for AprilTags)

    private final NetworkTableEntry
        ENTRY_LED_MODE,
        ENTRY_CAMERA_MODE,
        ENTRY_PIPELINE_SELECT,
        ENTRY_STREAM_MODE,
        ENTRY_SNAPSHOT,
        ENTRY_CROP_RECTANGLE;

    public Limelight (String cameraAddress, String cameraIP) {
        TABLE = NetworkTableInstance.getDefault().getTable("limelight-" + cameraAddress);
        ENTRY_TV =      TABLE.getEntry("tv");
        ENTRY_TX =      TABLE.getEntry("tx");
        ENTRY_TY =      TABLE.getEntry("ty");
        ENTRY_TA =      TABLE.getEntry("ta");
        ENTRY_TL =      TABLE.getEntry("tl");
        ENTRY_TSHORT =  TABLE.getEntry("tshort");
        ENTRY_TLONG =   TABLE.getEntry("tlong");
        ENTRY_THOR =    TABLE.getEntry("thor");
        ENTRY_TVERT =   TABLE.getEntry("tvert");
        ENTRY_GETPIPE = TABLE.getEntry("getpipe");
        ENTRY_JSON =    TABLE.getEntry("json");
        ENTRY_TCLASS =  TABLE.getEntry("tclass");
        ENTRY_TC =      TABLE.getEntry("tc");
        ENTRY_TPOSE =   TABLE.getEntry("targetpose_robotspace");
        ENTRY_RPOSE =   TABLE.getEntry("botpose");
        ENTRY_TID =     TABLE.getEntry("tid");

        ENTRY_LED_MODE          = TABLE.getEntry("ledMode");
        ENTRY_CAMERA_MODE       = TABLE.getEntry("camMode");
        ENTRY_PIPELINE_SELECT   = TABLE.getEntry("pipeline");
        ENTRY_STREAM_MODE       = TABLE.getEntry("stream");
        ENTRY_SNAPSHOT          = TABLE.getEntry("snapshot");
        ENTRY_CROP_RECTANGLE    = TABLE.getEntry("crop");

        cameraServer = new MjpegServer("limelight", "http://" + cameraIP, 5800);

        allLimelights.add(this);
    }

    public VideoSource getSource () {
        return cameraServer.getSource();
    }
    
    // Basic target recognition
   

    public boolean hasValidTarget () {
        return ENTRY_TV.getInteger(0) == 1;
    }
    
    public Optional<TargetData> getTarget () {
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
                ENTRY_TC.getDoubleArray(new double[3])
            ));
        }
    }

    public boolean hasAprilTag() {
        return ENTRY_TID.getDouble(0) != 0;
    }
    
    /**
     * Gets a {@code Pose3d} from a Translation(x, y, z), Rotation(roll, pitch, yaw) array
     */
    private static Pose3d getPoseFromArray (double[] array) {
        if (array.length != 6) return new Pose3d();
        return new Pose3d(
            array[0], array[1], array[2],                   // x, y, z
            new Rotation3d(array[3], array[4], array[5])    // roll, pitch, yaw
        );
    }
    
    public Optional<AprilTagData> getAprilTag () {
        if (hasAprilTag()) {
            return Optional.of(new AprilTagData(
                ENTRY_TID.getDouble(0),
                ENTRY_TX.getDouble(0),
                ENTRY_TY.getDouble(0),
                getPoseFromArray(ENTRY_TPOSE.getDoubleArray(new double[0])),
                getPoseFromArray(ENTRY_RPOSE.getDoubleArray(new double[0]))
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
        Pose3d targetPose,
        Pose3d robotPose
    ) { }
    
    // Misc. data
    
    public double getPipelineLatency () {
        return ENTRY_TL.getDouble(0);
    }
    
    public String getJSONDump () {
        return ENTRY_JSON.getString("{}");
    }

    public int getTargetID () {
        return (int)ENTRY_TID.getInteger(0);
    }
    
    public int getActivePipeline () {
        return (int)ENTRY_GETPIPE.getInteger(-1);
    }
    
    // Camera controls
    
    
    
    public void setLEDMode (LEDMode mode) {
        ENTRY_LED_MODE.setDouble(mode.mode);
    }
    
    public enum LEDMode {
        PIPELINE_DEFAULT    (0),
        FORCE_OFF           (1),
        FORCE_BLINK         (2),
        FORCE_ON            (3);
        
        private int mode;
        private LEDMode (int mode) {
            this.mode = mode;
        }
        
    }
    
    public void setCameraMode (CameraMode mode) {
        ENTRY_CAMERA_MODE.setDouble(mode.mode);
    }
    
    public enum CameraMode {
        VISION_PROCESSOR    (0),
        DRIVER_CAMERA       (1);
        
        private int mode;
        private CameraMode (int mode) {
            this.mode = mode;
        }
        
    }
    
    /**
     * Takes in @param pipelineIndex
     * Pipeline 0 is AprilTags
     * Pipeline 1 is Retroreflective tape 
     */
    public void setPipeline (int pipelineIndex) {
        ENTRY_PIPELINE_SELECT.setDouble(pipelineIndex);
    }
    
    public void setStreamMode (StreamMode mode) {
        ENTRY_STREAM_MODE.setDouble(mode.mode);
    }
    
    public enum StreamMode {
        STANDARD        (0),
        PiP_MAIN        (1),
        PiP_SECONDARY   (2);
        
        private int mode;
        private StreamMode (int mode) {
            this.mode = mode;
        }
        
    }
    
    private long lastSnapshotActionTime = 0;
    
    private boolean canTakeSnapshotAction () {
        return lastSnapshotActionTime + SNAPSHOT_RESET_MILLIS < System.currentTimeMillis();
    }
    
    private boolean canResetSnapshot () {
        return canTakeSnapshotAction() && ENTRY_SNAPSHOT.getDouble(0) == 1;
    }
    
    public boolean canTakeSnapshot () {
        return canTakeSnapshotAction() && ENTRY_SNAPSHOT.getDouble(1) == 0;
    }
    
    public void takeSnapshot () {
        if (canTakeSnapshot()) {
            lastSnapshotActionTime = System.currentTimeMillis();
            ENTRY_SNAPSHOT.setDouble(1);
        }
    }
    
    private void resetSnapshot () {
        if (canResetSnapshot()) {
            lastSnapshotActionTime = System.currentTimeMillis();
            ENTRY_SNAPSHOT.setDouble(0);
        }
    }
    
    public void setCropRectangle (double minX, double minY, double maxX, double maxY) {
        ENTRY_CROP_RECTANGLE.setDoubleArray(new double[]{ minX, minY, maxX, maxY });
    }
    
    private void updateInstance () {
        if (canResetSnapshot())
            resetSnapshot();
    }

    public static void update () {
        for (Limelight limelight : allLimelights) {
            limelight.updateInstance();
        }
    }
    
}

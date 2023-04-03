// package frc.robot.vision;

// import java.util.Optional;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoSink;
// import edu.wpi.first.cscore.VideoSource;
// import frc.robot.limelight.Limelight;

// public class Cameras {
    
//     private static Cameras instance;
    
//     public static Cameras getInstance () {
//         if (instance == null) instance = new Cameras();
//         return instance;
//     }
    
//     private final VideoSink cameraVideoSink;
    
//     private Camera actualActiveCamera, idealActiveCamera = Camera.INTAKE_CAMERA;
    
//     private Cameras () {
//         cameraVideoSink = CameraServer.addSwitchedCamera("Switchable Camera");
//     }
    
//     public enum Camera {
//         PAN_CAMERA      (1),
//         INTAKE_CAMERA   (Limelight.INTAKE_LIMELIGHT),
//         ARM_CAMERA      (Limelight.ARM_LIMELIGHT);
        
//         private final VideoSource camera;
//         private Camera (int deviceNum) {
//             this.camera = new UsbCamera(this.name(), deviceNum);
//         }
        
//         private Camera (Limelight limelight) {
//             camera = limelight.getSource();
//         }
//     }
    
//     public void setCamera (Camera camera) {
//         idealActiveCamera = camera;
//     }
    
//     public Optional<Camera> getActualActiveCamera () {
//         return Optional.ofNullable(actualActiveCamera);
//     }
    
//     private void updateActualActiveCamera (Camera camera) {
//         if (actualActiveCamera != camera) {
//             actualActiveCamera = camera;
//             cameraVideoSink.setSource(camera.camera);
//         }
//     }
    
//     private void switchToAnyConnectedCamera () {
//         // Switch to the first active camera
//         for (Camera camera : Camera.values()) {
//             if (camera.camera.isConnected()) {
//                 updateActualActiveCamera(camera);
//                 return;
//             }
//         }
//     }
    
//     public void periodicUpdate () {
//         // Switch to the ideal active camera if it is connected. Otherwise, switch to any connected camera
//         if (idealActiveCamera.camera.isConnected()) {
//             updateActualActiveCamera(idealActiveCamera);
//         } else {
//             switchToAnyConnectedCamera();
//         }
        
//     }
    
// }

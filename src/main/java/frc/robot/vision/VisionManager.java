// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.limelight.Limelight;
import frc.robot.limelight.Limelight.CameraMode;

/** Add your docs here. */
public class VisionManager {
    
    private static VisionManager instance;
    
    public static VisionManager getInstance () {
        if (instance == null) {
            instance = new VisionManager();
        }
        
        return instance;
    }
    
	private double ARM_OFFSET_DEGREES = 10;
	private MjpegServer camServer;
    
    private UsbCamera panCam = new UsbCamera("Pan Camera", 0);
    
	private VisionManager () {
		camServer = CameraServer.addSwitchedCamera("Arm Camera stream");
		CameraServer.startAutomaticCapture(Limelight.INTAKE_LIMELIGHT.getSource());
        
        Limelight.INTAKE_LIMELIGHT.setCameraMode(CameraMode.DRIVER_CAMERA);
        Limelight.ARM_LIMELIGHT.setCameraMode(CameraMode.DRIVER_CAMERA);
	}
    
	public void manageCamStreams (Rotation2d armRotation) {
        double armAngle = armRotation.getDegrees();
		if (armAngle <= ARM_OFFSET_DEGREES) {
			camServer.setSource(panCam);
		}
		if (armAngle > ARM_OFFSET_DEGREES) {
			camServer.setSource(Limelight.ARM_LIMELIGHT.getSource());
		}
	}
    
}

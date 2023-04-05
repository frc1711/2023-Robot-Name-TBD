// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.limelight.Limelight;

/** Add your docs here. */
public class VisionManager {

	private double ARM_OFFSET_DEGREES = 10;
	private MjpegServer camServer;

	private VisionManager () {
		camServer = CameraServer.addSwitchedCamera("Arm Camera stream");
		CameraServer.startAutomaticCapture(Limelight.INTAKE_LIMELIGHT.getSource());
	}

	public void manageCamStreams (double armAngle) {
		if (armAngle <= ARM_OFFSET_DEGREES) {
			camServer.setSource(new UsbCamera("Pan Camera", 0));
		}
		if (armAngle > ARM_OFFSET_DEGREES) {
			camServer.setSource(Limelight.ARM_LIMELIGHT.getSource());
		}
	}
}

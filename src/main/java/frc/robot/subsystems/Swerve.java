<<<<<<< Updated upstream
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
=======
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
>>>>>>> Stashed changes

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import claw.subsystems.SubsystemCLAW;


public class Swerve extends SubsystemCLAW {

  public Swerve() {}

  public CommandBase exampleMethodCommand() {

<<<<<<< Updated upstream
    return runOnce(
        () -> {

        });
  }


  public boolean exampleCondition() {

    return false;
=======
	private final Translation2d flPosition, frPosition, rlPosition, rrPosition;
	public final SwerveDriveKinematics kinematics;
	public SwerveModuleState[] modules;
	public ChassisSpeeds speed;
  
  //Create an array of the SwerveModuleStates using SwerveDriveKinematics and an empty instance of ChassisSpeeds
  //TODO: Measure Swerve Module distances for Translations
  private Swerve() {
	gyro = new AHRS();
	flPosition = new Translation2d();
	frPosition = new Translation2d();
	rlPosition = new Translation2d();
	rrPosition = new Translation2d();
	kinematics = new SwerveDriveKinematics(flPosition, frPosition, rlPosition, rrPosition);
	speed = new ChassisSpeeds();
	modules = kinematics.toSwerveModuleStates(speed);
  }

  //return angle of a singular swerve module
  public Rotation2d getAngle (SwerveModuleState swerveModule) {
	return swerveModule.angle;
  }
  /*Create a new instance of ChassisSpeeds using directional inputs, then use kinematics 
  to make an array of SwerveModuleStates which represent the target speed*/
  public void drive(double xInput, double yInput, double theta) {
	ChassisSpeeds targetSpeeds = new ChassisSpeeds(xInput, yInput, theta);
	SwerveModuleState[] targetStates = swerveInstance.kinematics.toSwerveModuleStates(targetSpeeds);
	modules[0] = SwerveModuleState.optimize(targetStates[0], swerveInstance.getAngle(swerveInstance.modules[0]));
	modules[1] = SwerveModuleState.optimize(targetStates[1], swerveInstance.getAngle(swerveInstance.modules[1]));
	modules[2] = SwerveModuleState.optimize(targetStates[2], swerveInstance.getAngle(swerveInstance.modules[2]));
	modules[3] = SwerveModuleState.optimize(targetStates[3], swerveInstance.getAngle(swerveInstance.modules[3]));
  }

  public void stop () {
	speed = new ChassisSpeeds();
  }

  public boolean canStartTeleop() {
    if (DriverStation.isTeleopEnabled()) return true;
    else return false;
>>>>>>> Stashed changes
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}

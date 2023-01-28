// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.RotationalPID;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.Vision;

public class AprilTags extends CommandBase {

  Swerve swerveDrive;
  Vision vision;
  RotationalPID rotationPID = new RotationalPID("LimelightRotationPID", .03, 0, 0, .5);

  public AprilTags(
    Swerve swerveDrive,
    Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    addRequirements(swerveDrive);
  }

  //Get Swerve to drive to the tag given as input
  Transform3d transform;
  public void driveToTag () {
    if (vision.seesTarget()) swerveDrive.moveFieldRelative(new ChassisSpeeds(0, vision.getDistance(), 0));
  }

  boolean isFacingTag;
  boolean sawTag;
  public boolean turnToTag () {
    isFacingTag = false;

    if (vision.seesTarget()) { 
      Rotation2d robotPosition = Rotation2d.fromDegrees(swerveDrive.getRobotYaw());
      Rotation2d tagPosition = Rotation2d.fromDegrees(vision.getHorizontalOffset() + swerveDrive.getRobotYaw());
      swerveDrive.moveFieldRelative(new ChassisSpeeds(0, 0, rotationPID.calculate(robotPosition, tagPosition)));
      isFacingTag = true;
    }
    
    else if (!vision.seesTarget()) swerveDrive.stop();
    return isFacingTag;  
  }

  // Check if any cameras have been activated, if true, continue normally, if false start a new camera
  @Override
  public void initialize() {
    swerveDrive.stop();
  }

  // Search for AprilTags on the camera. If any are found, drive toward them
  @Override
  public void execute() {
    if (vision.seesTarget() && Math.abs(vision.getHorizontalOffset()) > vision.maxOffsetDegrees) turnToTag();
    // else if (vision.seesTarget() && vision.getHorizontalOffset() < vision.maxOffsetDegrees) driveToTag();
    else if (!vision.seesTarget()) swerveDrive.stop();
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

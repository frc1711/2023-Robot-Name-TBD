// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.Vision;

public class AprilTags extends CommandBase {

  Swerve swerveDrive;
  Vision vision;

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
  public boolean turnToTag () {
    isFacingTag = false;
    if (vision.seesTarget()) { 
      swerveDrive.moveFieldRelative(new ChassisSpeeds(0, 0, vision.getHorizontalOffset()));
      isFacingTag = true;
    }
    return isFacingTag;  
  }

  // Check if any cameras have been activated, if true, continue normally, if false start a new camera
  @Override
  public void initialize() {
    swerveDrive.stop();
    if (!vision.isStart()) vision.start();
  }

  // Search for AprilTags on the camera. If any are found, drive toward them
  @Override
  public void execute() {
    if (vision.seesTarget() && vision.getHorizontalOffset() > vision.maxOffsetDegrees) turnToTag();
    else if (vision.seesTarget() && vision.getHorizontalOffset() < vision.maxOffsetDegrees) driveToTag();
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
    vision.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
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
  public void driveToTag (AprilTagDetection detection) {
    transform = vision.getTagDistance(detection);
    swerveDrive.drive(transform.getX(), transform.getY(), transform.getRotation().getAngle());
  }

  @Override
  public void initialize() {
    swerveDrive.stop();
    vision.start();
  }

  @Override
  public void execute() {
    AprilTagDetection detection = vision.detectAprilTags();
    if (detection != null) driveToTag(detection);
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

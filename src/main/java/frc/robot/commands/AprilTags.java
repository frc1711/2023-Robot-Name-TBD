// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.RotationalPID;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.NumericDebouncer;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.Vision;

public class AprilTags extends CommandBase {

  private static final double MAX_OFFSET_DEGREES = 5;

  private Swerve swerveDrive;
  private Vision vision;
  private RotationalPID rotationPID = new RotationalPID(.16, 0, 0, MAX_OFFSET_DEGREES);
  private PIDController distancePID = new PIDController(1, 0, 0);
  private NumericDebouncer rotationMeasurement = new NumericDebouncer (new Debouncer(0.2, DebounceType.kFalling));
  private NumericDebouncer distanceMeasurement = new NumericDebouncer(new Debouncer(0.2, DebounceType.kFalling));
  private SlewRateLimiter turnRateLimiter = new SlewRateLimiter(8);

  public AprilTags(
    Swerve swerveDrive,
    Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    addRequirements(swerveDrive);
    distancePID.setTolerance(1);
  }

  //Get Swerve to drive to the tag given as input
  public double getDriveToTag (Optional<Double> targetDistance) {
    double driveSpeed;

    if (targetDistance.isPresent()) driveSpeed = distancePID.calculate(0, targetDistance.get());
    else driveSpeed = 0;

    return driveSpeed;
  }

  public double getTurnToTag (Optional<Double> targetRotationDegrees) {
    double turnSpeed;

    if (targetRotationDegrees.isPresent()) {
      Rotation2d targetRotation = Rotation2d.fromDegrees(targetRotationDegrees.get());
      Rotation2d robotRotation = swerveDrive.getRobotRotation();
      turnSpeed = rotationPID.calculate(robotRotation, targetRotation);
    } else {
      turnSpeed = 0;
    }

    return turnRateLimiter.calculate(turnSpeed);
  }

  // Check if any cameras have been activated, if true, continue normally, if false start a new camera
  @Override
  public void initialize() {
    swerveDrive.stop();
  }

  // Search for AprilTags on the camera. If any are found, drive toward them
  @Override
  public void execute() {
    Optional<Double> targetRotation;
    Optional<Double> targetDistance;

    if (vision.seesTarget() && vision.isTargetFriendly()) {
      Rotation2d tagAbsRotation = swerveDrive.getRobotRotation().plus(Rotation2d.fromDegrees((Limelight.ARM_LIMELIGHT.getTarget()).get().horizontalOffset()));
      targetRotation = rotationMeasurement.calculate(Optional.of(tagAbsRotation.getDegrees()));
      targetDistance = distanceMeasurement.calculate(Optional.of(vision.getDistance()));
    } else {
      targetRotation = rotationMeasurement.calculate(Optional.empty());
      targetDistance = distanceMeasurement.calculate(Optional.empty());
    }

    double turnRate = getTurnToTag(targetRotation);
    double driveSpeed = getDriveToTag(targetDistance);

    swerveDrive.moveRobotRelative(new ChassisSpeeds(driveSpeed, 0, turnRate));
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

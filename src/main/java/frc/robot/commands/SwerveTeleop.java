// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveTeleop extends CommandBase {
  private final Swerve swerveDrive;
  private final DoubleSupplier xInput, yInput, theta;

  public SwerveTeleop(Swerve swerveDrive,
                      DoubleSupplier xInput,
                      DoubleSupplier yInput,
                      DoubleSupplier theta) {
    this.swerveDrive = swerveDrive;
    this.xInput = xInput;
    this.yInput = yInput;
    this.theta = theta;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    swerveDrive.stop();
  }

  //TODO: Add safety features (safety brake, slow mode, etc)
  @Override
  public void execute() {
    swerveDrive.drive(xInput.getAsDouble()
                      , yInput.getAsDouble()
                      , theta.getAsDouble());
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.auton.BalanceCommand;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final Swerve swerveDrive = Swerve.getInstance();

  private final XboxController driverController =
      new XboxController(0);

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
      new DriveCommand(swerveDrive, null, () -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getRightX());
  }

  public Command getAutonomousCommand() {
    return new BalanceCommand(null);
  }
}

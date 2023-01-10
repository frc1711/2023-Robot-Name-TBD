// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.IDMap.OperatorConstants;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final Swerve swerveDrive = Swerve.getInstance();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {

    new Trigger(swerveDrive::canStartTeleop)
        .onTrue(new SwerveTeleop(swerveDrive, 
                                  () -> driverController.getLeftX(),
                                  () -> driverController.getLeftY(),
                                  () -> driverController.getRightX()));
  }

  // public Command getAutonomousCommand() {}
}

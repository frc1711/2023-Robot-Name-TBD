// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CentralCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.auton.BalanceCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final Swerve swerveDrive = Swerve.getInstance();
  private final Arm arm = Arm.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Conveyor conveyor = Conveyor.getInstance();

  private final XboxController driverController =
      new XboxController(0);
  private final XboxController secondaryController = 
      new XboxController(1);

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
      new DriveCommand(swerveDrive, null, () -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getRightX());
      new CentralCommand(arm, conveyor, intake, null, null, null, null, () -> secondaryController.getBButton(), () -> secondaryController.getLeftStickButton(), () -> secondaryController.getAButton(), );
  }

  public Command getAutonomousCommand() {
    return new BalanceCommand(swerveDrive);
  }
}

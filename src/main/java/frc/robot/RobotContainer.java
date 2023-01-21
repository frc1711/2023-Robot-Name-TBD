// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.auton.Balance;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    
    private final XboxController driveController = new XboxController(0);
    
    private final Swerve swerveSubsystem = new Swerve();
    
    private final DriveCommand driveCommand = new DriveCommand(
        swerveSubsystem,
        driveController::getLeftX,
        driveController::getLeftY,
        driveController::getRightX);
    
    public RobotContainer () {
        putSendable("Swerve Subsystem", swerveSubsystem);
        swerveSubsystem.setDefaultCommand(driveCommand);
    }
    
    public static void putSendable (String title, Sendable sendable) {
        Shuffleboard.getTab("Testing").add(title, sendable);
    }
    
    public Command getAutonomousCommand () {
        // This can return null to not run a command
        return new Balance(swerveSubsystem);
    }
    
}

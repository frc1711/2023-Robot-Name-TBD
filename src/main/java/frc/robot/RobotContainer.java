// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmControlCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.auton.BalanceCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainer {
    
    private final XboxController
        driveController = new XboxController(0),
        systemController = new XboxController(1);
    
    private final Swerve swerveSubsystem = new Swerve();
    private final Conveyor conveyorSubsystem = Conveyor.getInstance();
    private final Intake intakeSubsystem = Intake.getInstance();
    private final Arm armSubsystem = Arm.getInstance();
    
    private final DriveCommand driveCommand = new DriveCommand(
        swerveSubsystem,
        driveController::getStartButton,
        driveController::getLeftX,
        driveController::getLeftY,
        driveController::getRightX,
        driveController::getLeftStickButton,
        () -> driveController.getLeftTriggerAxis() > 0.8 && driveController.getRightTriggerAxis() > 0.8
    );
    
    private final TeleopIntake intakeCommand = new TeleopIntake(
        conveyorSubsystem,
        intakeSubsystem,
        systemController::getAButton,
        systemController::getBButton
    );
    
    private final ArmControlCommand armCommand = new ArmControlCommand(
        armSubsystem,
        () -> -systemController.getLeftY(),
        systemController::getLeftBumper,
        systemController::getRightBumper
    );
    
    public RobotContainer () {
        swerveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
        conveyorSubsystem.setDefaultCommand(intakeCommand);
        armSubsystem.setDefaultCommand(armCommand);
    }
    
    public static void putConfigSendable (String title, Sendable sendable) {
        // TODO: A temporary solution for adding sendables to the shuffleboard
        Shuffleboard.getTab("Config Board").add(title, sendable);
    }
    
    public static void putConfigCommand (String name, CommandBase command, boolean canRunWhenDisabled) {
        putConfigSendable(name, command.withName(name).ignoringDisable(true));
    }
    
    public Command getAutonomousCommand () {
        // This can return null to not run a command
        return new BalanceCommand(null);
    }
    
}

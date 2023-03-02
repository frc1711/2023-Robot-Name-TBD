// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmControlCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TeleopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
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
        () -> driveController.getLeftBumper(),
        () -> driveController.getRightBumper()
    );
    
    private final ArmControlCommand armCommand = new ArmControlCommand(
        armSubsystem,
        () -> -systemController.getLeftY(),
        systemController::getAButton,
        systemController::getXButton,
        systemController::getYButton,
        
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
        return null;
        // return swerveSubsystem.getControllerCommand(new Trajectory(Arrays.asList(
        //     new State(0, 0, 1, new Pose2d(), 0),
        //     new State(1, 1, 0, new Pose2d(), 0),
        //     new State(2, 0, 0, new Pose2d(1, 0, Rotation2d.fromDegrees(0)), 0)
        // )));
    }
    
}

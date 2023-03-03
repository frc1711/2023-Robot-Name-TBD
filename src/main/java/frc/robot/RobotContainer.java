// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmControlCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.auton.BalanceCommandAuton;
import frc.robot.commands.auton.DriveIntakeAuton;
import frc.robot.commands.auton.PlaceAndBalanceAuton;
import frc.robot.commands.auton.PlaceGamePieceTest;
import frc.robot.commands.auton.PlaceItemAuton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainer {
    
    private final SendableChooser<Supplier<Command>> autonChooser = new SendableChooser<>();
    
    private final XboxController
        driveController = new XboxController(0),
        systemController = new XboxController(1);
    
    private final Swerve swerveSubsystem = Swerve.getInstance();
    private final Conveyor conveyorSubsystem = Conveyor.getInstance();
    private final Intake intakeSubsystem = Intake.getInstance();
    private final Arm armSubsystem = Arm.getInstance();
    private final Claw clawSubsystem = new Claw();
    
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
        clawSubsystem,
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
        
        configAutonChooser();
    }
    
    public static void putConfigSendable (String title, Sendable sendable) {
        // TODO: A temporary solution for adding sendables to the shuffleboard
        Shuffleboard.getTab("Config Board").add(title, sendable);
    }
    
    public static void putConfigCommand (String name, CommandBase command, boolean canRunWhenDisabled) {
        putConfigSendable(name, command.withName(name).ignoringDisable(true));
    }
    
    private void configAutonChooser () {
        autonChooser.addOption("Balance", () -> new BalanceCommandAuton());
        autonChooser.addOption("Drive Intake", () -> new DriveIntakeAuton(swerveSubsystem));
        autonChooser.addOption("Place Item", () -> new PlaceItemAuton());
        autonChooser.addOption("Place and Balance", () -> new PlaceAndBalanceAuton());
        autonChooser.addOption("TEST", () -> new PlaceGamePieceTest(armSubsystem, clawSubsystem, swerveSubsystem, ArmPosition.MIDDLE));
        putConfigSendable("AUTON SELECT", autonChooser);
    }
    
    /**
     * TODO: PRE-MATCH TO-DOS
     * 1. Set the ArmPositions so they work
     * 2. Test auton
     * 3. Merge with safe-claw-dev branch
     *    - Test conveyor solo code
     *    - Test arm current stop code through RCT
     * 4. 
     */
    
    public Command getAutonomousCommand () {
        return autonChooser.getSelected().get();
        // return new PlaceGamePieceTest(armSubsystem, clawSubsystem, swerveSubsystem, ArmPosition.MIDDLE);
        // return swerveSubsystem.getControllerCommand(new Trajectory(Arrays.asList(
        //     new State(0, 0, 1, new Pose2d(), 0),
        //     new State(1, 1, 0, new Pose2d(), 0),
        //     new State(2, 0, 0, new Pose2d(1, 0, Rotation2d.fromDegrees(0)), 0)
        // )));
    }
    
}

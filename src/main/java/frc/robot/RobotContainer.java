// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmControlCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.auton.BalanceCommandAuton;
import frc.robot.commands.auton.PlaceAndBalanceAuton;
import frc.robot.commands.auton.PlaceAndTaxi;
import frc.robot.commands.auton.TaxiAuton;
import frc.robot.commands.auton.vision.DriveRelativeToAprilTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Supplier;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        () -> driveController.getLeftTriggerAxis() > 0.8 && driveController.getRightTriggerAxis() > 0.8,
        driveController::getXButton,
        
        () -> {
            int povRead = driveController.getPOV();
            return povRead == -1 ? Optional.empty() : Optional.of(Double.valueOf(povRead));
        }
    );
    
    private final TeleopIntake intakeCommand = new TeleopIntake(
        conveyorSubsystem,
        intakeSubsystem,
        () -> driveController.getLeftBumper(),
        () -> driveController.getRightBumper(),
        () -> systemController.getRightY() < -0.5,
        () -> systemController.getRightY() > 0.5,
        () -> systemController.getLeftTriggerAxis() > 0.5
    );
    
    private final ArmControlCommand armCommand = new ArmControlCommand(
        armSubsystem,
        clawSubsystem,
        
        () -> -systemController.getLeftY(),
        systemController::getAButton,
        systemController::getXButton,
        systemController::getYButton,
        systemController::getBButton,
        
        systemController::getLeftBumper,
        systemController::getRightBumper
    );
    
    public RobotContainer () {
        swerveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
        conveyorSubsystem.setDefaultCommand(intakeCommand);
        armSubsystem.setDefaultCommand(armCommand);
        
        configAutonChooser();
        
        new Trigger(systemController::getStartButton).whileTrue(
            new ConditionalCommand(
                new DriveRelativeToAprilTag(
                    swerveSubsystem,
                    new Transform2d(
                        new Translation2d(
                            Units.inchesToMeters(28),
                            0
                        ),
                        Rotation2d.fromDegrees(180)
                    )
                ),
                new InstantCommand(()->{}),
                () -> armSubsystem.getArmRotation().getDegrees() > 70
            )
        );
        
    }

    public static void putConfigSendable (String title, Sendable sendable) {
        // TODO: A temporary solution for adding sendables to the shuffleboard
        Shuffleboard.getTab("Config Board").add(title, sendable);
    }
    
    public static void putConfigCommand (String name, CommandBase command, boolean canRunWhenDisabled) {
        putConfigSendable(name, command.withName(name).ignoringDisable(true));
    }
    
    private void configAutonChooser () {
        autonChooser.addOption("Balance", () -> new BalanceCommandAuton(swerveSubsystem, false));
        autonChooser.addOption("Taxi only", () -> new TaxiAuton(swerveSubsystem));
        autonChooser.addOption("Place and Balance", () -> new PlaceAndBalanceAuton(swerveSubsystem, armSubsystem, clawSubsystem, ArmPosition.HIGH));
        autonChooser.addOption("Place and Taxi (Octopus)", () -> new PlaceAndTaxi(swerveSubsystem, armSubsystem, clawSubsystem, ArmPosition.HIGH, DriverStation.getAlliance()));
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
        Supplier<Command> selectedAuton = autonChooser.getSelected();
        if (selectedAuton == null) return null;
        return selectedAuton.get().withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        // return new PlaceGamePieceTest(armSubsystem, clawSubsystem, swerveSubsystem, ArmPosition.MIDDLE);
        // return swerveSubsystem.getControllerCommand(new Trajectory(Arrays.asList(
        //     new State(0, 0, 1, new Pose2d(), 0),
        //     new State(1, 1, 0, new Pose2d(), 0),
        //     new State(2, 0, 0, new Pose2d(1, 0, Rotation2d.fromDegrees(0)), 0)
        // )));
    }
    
}

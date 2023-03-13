package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Claw.ClawMovement;
import frc.robot.subsystems.swerve.Swerve;

public class PlaceGamePieceTest extends SequentialCommandGroup {
    
    public PlaceGamePieceTest (Arm arm, Claw claw, Swerve swerve, ArmPosition armScorePosition) {
        super(
            
            // Home the claw to fully opened
            new InstantCommand(claw::homeAsFullyOpen, claw),
            
            // Grab the game piece
            new ControlClawCommand(claw, ClawMovement.GRAB),
            
            // Move the arm
            new MoveArmCommand(arm, claw, ArmPosition.HIGH, ClawMovement.GRAB),
            
            // Drive toward grid
            new DriveStraightCommand(swerve, true, 1, 0.8, true),
            
            // Release the game piece
            new ControlClawCommand(claw, ClawMovement.RELEASE),
            
            // Wait just a moment before coming back
            new WaitCommand(0.2),
            
            // Drive away from grid
            new DriveStraightCommand(swerve, false, 1, 0.8, true),
            
            // Move the arm back in
            new MoveArmCommand(arm, claw, ArmPosition.STOWED, ClawMovement.GRAB)
            
        );
    }
    
    private static Command withControlClaw (Command command, Claw claw, ClawMovement clawControl) {
        return new ParallelDeadlineGroup(
            command,
            new RunCommand(() -> claw.operateClaw(clawControl), claw)
        );
    }
    
}

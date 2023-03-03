package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            new ControlClawCommand(claw, ClawMovement.GRAB)
            
            // // Move the arm to the correct position to score
            // ,withControlClaw(
            //     new MoveArmCommand(arm, armScorePosition),
            //     claw,
            //     ClawMovement.GRAB
            // ),
            
            // // // Move swerve forwards
            // // withControlClaw(
            // //     new TimedDriveCommand(
            // //         swerve,
            // //         1,
            // //         0,
            // //         2,
            // //         2
            // //     ),
            // //     claw,
            // //     ClawMovement.GRAB
            // // )
            
            // // Release the game piece
            // new ControlClawCommand(claw, ClawMovement.RELEASE),
            
            // // // Move swerve backwards
            // // withControlClaw(
            // //     new TimedDriveCommand(
            // //         swerve,
            // //         -1,
            // //         0,
            // //         2,
            // //         2
            // //     ),
            // //     claw,
            // //     ClawMovement.RELEASE
            // // ),
            
            // // Stow the arm
            // withControlClaw(
            //     new MoveArmCommand(arm, ArmPosition.STOWED),
            //     claw,
            //     ClawMovement.RELEASE
            // )
            
        );
    }
    
    private static Command withControlClaw (Command command, Claw claw, ClawMovement clawControl) {
        return new ParallelDeadlineGroup(
            command,
            new RunCommand(() -> claw.operateClaw(clawControl), claw)
        );
    }
    
}

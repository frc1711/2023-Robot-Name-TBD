package frc.robot.commands.auton.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auton.MoveArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Claw.ClawMovement;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Drive the robot to the high or mid position for cube scoring based on whatever apriltag is visible.
 */
public class AutoCubeSetup extends ParallelCommandGroup {
    
    public AutoCubeSetup (Swerve swerve, Arm arm, Claw claw, boolean scoreHigh) {
        
        Command driveCommand = scoreHigh
            ? DriveRelativeToAprilTag.getHighCentering(swerve)
            : DriveRelativeToAprilTag.getMidCentering(swerve);
        
        ArmPosition armPosition = scoreHigh
            ? ArmPosition.HIGH
            : ArmPosition.MIDDLE;
        
        addCommands(
            driveCommand,
            new MoveArmCommand(arm, claw, armPosition, ClawMovement.NONE)
        );
        
    }
    
}

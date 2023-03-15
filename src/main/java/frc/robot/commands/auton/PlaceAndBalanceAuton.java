package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.Swerve;

public class PlaceAndBalanceAuton extends SequentialCommandGroup {
    
    public PlaceAndBalanceAuton (Swerve swerve, Arm arm, Claw claw, ArmPosition armScorePosition) {
        super(
            new PlaceGamePieceSimple(arm, claw, swerve, armScorePosition),
            new BalanceCommandAuton(swerve, true)
        );
    }
    
}

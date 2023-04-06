package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.Swerve;

public class WireGuardPlaceAndTaxi extends SequentialCommandGroup {
    
    public WireGuardPlaceAndTaxi (Swerve swerve, Arm arm, Claw claw) {
        super(
            new PlaceGamePieceSimple(arm, claw, swerve, ArmPosition.HIGH),
            new DriveStraightCommand(swerve, false, 3, 1.5, true)
        );
    }
    
}

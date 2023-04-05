package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.Swerve;

public class WireGuardPlaceAndIntake extends SequentialCommandGroup {
    
    public WireGuardPlaceAndIntake (Swerve swerve, Arm arm, Claw claw, Intake intake) {
        super(
            new PlaceGamePieceSimple(arm, claw, swerve, ArmPosition.HIGH),
            new DriveStraightCommand(swerve, false, 3, 1.5, true)
                .alongWith(new WaitCommand(2).andThen(new RunTimedIntake(intake, 3)))
        );
    }
    
}

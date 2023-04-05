package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.swerve.Swerve;

// Octopus, clear side auto
public class PlaceAndTaxi extends SequentialCommandGroup {
    
    public PlaceAndTaxi (Swerve swerve, Arm arm, Claw claw, ArmPosition scorePosition, Alliance alliance) {
        super(
            new PlaceGamePieceSimple(arm, claw, swerve, scorePosition),
            new TimedDriveCommand(swerve, 0, 2 * (alliance.equals(Alliance.Red) ? 1 : -1), 12, 0.65, true),
            new DriveStraightCommand(swerve, false, 3, 1.2, true)
        );
    }
    
}

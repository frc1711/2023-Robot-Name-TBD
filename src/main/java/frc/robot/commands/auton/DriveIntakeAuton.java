package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.swerve.Swerve;

public class DriveIntakeAuton extends WrapperCommand {
    
    public DriveIntakeAuton (Swerve swerve) {
        super(
            new TimedDriveCommand(swerve, 1, 0, 1, 3)
        );
    }
    
}

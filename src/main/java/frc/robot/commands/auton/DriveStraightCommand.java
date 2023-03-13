package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.swerve.Swerve;

public class DriveStraightCommand extends WrapperCommand {
    
    private static final double DRIVE_SPEED = 2;
    
    public DriveStraightCommand (Swerve swerve, boolean isForward, double time, double speedMult) {
        super(
            new TimedDriveCommand(swerve, isForward ? DRIVE_SPEED*speedMult : -DRIVE_SPEED*speedMult, 0, 12, time)
        );
    }
    
}

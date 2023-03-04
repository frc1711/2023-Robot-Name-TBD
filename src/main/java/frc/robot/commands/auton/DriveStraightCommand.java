package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.swerve.Swerve;

public class DriveStraightCommand extends WrapperCommand {
    
    private static final double DRIVE_SPEED = 2;
    
    public DriveStraightCommand (Swerve swerve, boolean isForward, double time, boolean withTurnCorrection) {
        super(
            new TimedDriveCommand(swerve, isForward ? DRIVE_SPEED : -DRIVE_SPEED, 0, 12, time, withTurnCorrection)
        );
    }
    
}

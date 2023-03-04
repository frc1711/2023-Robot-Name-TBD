package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.swerve.Swerve;

public class TaxiAuton extends WrapperCommand {
    
    public TaxiAuton (Swerve swerve) {
        super(
            new DriveStraightCommand(swerve, true, 2.3, true)
        );
    }
    
}

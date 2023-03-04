package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

public class BalanceCommandAuton extends SequentialCommandGroup {
    
    public BalanceCommandAuton (Swerve swerve) {
        super(
            new DriveStraightCommand(swerve, true, 3, true),
            new BalanceSlowCommand(swerve),
            new RunCommand(swerve::xMode, swerve).withTimeout(2)
        );
    }
    
}

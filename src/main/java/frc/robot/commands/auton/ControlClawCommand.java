package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMovement;

public class ControlClawCommand extends CommandBase {
    
    private final Claw claw;
    private final ClawMovement movement;
    
    public ControlClawCommand (Claw claw, ClawMovement movement) {
        this.claw = claw;
        this.movement = movement;
        addRequirements(claw);
    }
    
    @Override
    public void initialize () {
        claw.operateClaw(ClawMovement.NONE);
    }
    
    @Override
    public void execute () {
        claw.operateClaw(movement);
    }
    
    @Override
    public void end (boolean interrupted) {
        claw.operateClaw(ClawMovement.NONE);
    }
    
    @Override
    public boolean isFinished () {
        switch (movement) {
            case GRAB:
                return claw.isFullyGrabbing();
            case RELEASE:
                return claw.isFullyReleased();
            case NONE:
                return true;
            default:
                return true;
        }
    }
    
}

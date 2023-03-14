package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMovement;

public class ControlClawCommand extends CommandBase {
    
    private final Claw claw;
    private final Supplier<Rotation2d> armRotationSupplier;
    private final ClawMovement movement;
    
    public ControlClawCommand (Claw claw, ClawMovement movement, Supplier<Rotation2d> armRotationSupplier) {
        this.claw = claw;
        this.movement = movement;
        this.armRotationSupplier = armRotationSupplier;
        addRequirements(claw);
    }
    
    @Override
    public void initialize () {
        claw.operateClaw(ClawMovement.NONE, armRotationSupplier.get());
    }
    
    @Override
    public void execute () {
        claw.operateClaw(movement, armRotationSupplier.get());
    }
    
    @Override
    public void end (boolean interrupted) {
        claw.operateClaw(ClawMovement.NONE, armRotationSupplier.get());
    }
    
    @Override
    public boolean isFinished () {
        switch (movement) {
            case GRAB:
                return claw.isFullyGrabbing();
            case RELEASE:
                return claw.isFullyReleased(armRotationSupplier.get());
            case NONE:
                return true;
            default:
                return true;
        }
    }
    
}

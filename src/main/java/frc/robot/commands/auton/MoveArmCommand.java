package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Claw.ClawMovement;

public class MoveArmCommand extends CommandBase {
    
    private final Arm arm;
    private final Claw claw;
    private final Rotation2d targetRotation;
    private final ClawMovement clawMovement;
    
    public MoveArmCommand (Arm arm, Claw claw, ArmPosition targetPosition, ClawMovement clawMovement) {
        this(arm, claw, targetPosition.rotation, clawMovement);
    }
    
    public MoveArmCommand (Arm arm, Claw claw, Rotation2d targetRotation, ClawMovement clawMovement) {
        this.arm = arm;
        this.claw = claw;
        this.targetRotation = targetRotation;
        this.clawMovement = clawMovement;
        addRequirements(arm, claw);
    }
    
    @Override
    public void initialize () {
        arm.stop();
        claw.operateClaw(ClawMovement.NONE);
    }
    
    @Override
    public void execute () {
        arm.setArmSpeed(arm.getSpeedToMoveToRotation(targetRotation));
        claw.operateClaw(clawMovement);
    }
    
    @Override
    public void end (boolean interrupted) {
        arm.stop();
        claw.operateClaw(ClawMovement.NONE);
    }
    
    @Override
    public boolean isFinished () {
        return Math.abs(arm.getArmRotation().minus(targetRotation).getDegrees()) < 2;
    }
    
}

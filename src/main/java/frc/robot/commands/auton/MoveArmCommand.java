package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class MoveArmCommand extends CommandBase {
    
    private final Arm arm;
    private final Rotation2d targetRotation;
    
    public MoveArmCommand (Arm arm, ArmPosition targetPosition) {
        this(arm, targetPosition.rotation);
    }
    
    public MoveArmCommand (Arm arm, Rotation2d targetRotation) {
        this.arm = arm;
        this.targetRotation = targetRotation;
        addRequirements(arm);
    }
    
    @Override
    public void initialize () {
        arm.stop();
    }
    
    @Override
    public void execute () {
        arm.moveArmToRotation(targetRotation);
    }
    
    @Override
    public void end (boolean interrupted) {
        arm.stop();
    }
    
    @Override
    public boolean isFinished () {
        return Math.abs(arm.getArmRotation().minus(targetRotation).getDegrees()) < 2;
    }
    
}

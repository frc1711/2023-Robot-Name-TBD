package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.ClawMovement;

public class ArmControlCommand extends CommandBase {
    
    private final Arm arm;
    private final DoubleSupplier armControl;
    private final BooleanSupplier armToLow, armToMid, armToHigh, grabControl, releaseControl;
    
    private final SlewRateLimiter armSpeedFilter = new SlewRateLimiter(1, -1, 0);
    private final Transform armSpeedTransform =
        InputTransform.getInputTransform(InputTransform.THREE_HALVES_CURVE, 0.1)
        .then(armSpeedFilter::calculate);
    
    private final Transform armOffsetFromSetpointToInputSpeed =
        Transform.NEGATE
        .then((Transform)(x -> x/40))
        .then(Transform.clamp(-1, 1))
        .then((Transform)(x -> 9*x));
    
    public ArmControlCommand (
        Arm arm,
        DoubleSupplier armControl,
        BooleanSupplier armToLow,
        BooleanSupplier armToMid,
        BooleanSupplier armToHigh,
        
        BooleanSupplier grabControl,
        BooleanSupplier releaseControl
    ) {
        this.arm = arm;
        this.armControl = armControl;
        this.armToLow = armToLow;
        this.armToMid = armToMid;
        this.armToHigh = armToHigh;
        
        this.grabControl = grabControl;
        this.releaseControl = releaseControl;
        addRequirements(arm);
    }
    
    @Override
    public void initialize () {
        arm.stop();
    }
    
    @Override
    public void execute () {
        
        // Set claw movement
        if (!arm.hasClawBeenHomed()) {
            arm.runClawHomingSequence();
        } else if (grabControl.getAsBoolean()) {
            arm.operateClaw(ClawMovement.GRAB);
        } else if (releaseControl.getAsBoolean()) {
            arm.operateClaw(ClawMovement.RELEASE);
        } else {
            arm.operateClaw(ClawMovement.NONE);
        }
        
        // Set arm movement
        double armInputSpeed = 0;
        if (armToLow.getAsBoolean() || armToMid.getAsBoolean() || armToHigh.getAsBoolean()) {
            
            ArmPosition armSetPosition;
            if (armToLow.getAsBoolean()) {
                armSetPosition = ArmPosition.LOW;
            } else if (armToMid.getAsBoolean()) {
                armSetPosition = ArmPosition.MIDDLE;
            } else {
                armSetPosition = ArmPosition.HIGH;
            }
            
            armInputSpeed = armOffsetFromSetpointToInputSpeed.apply(
                arm.getArmRotation().minus(armSetPosition.rotation).getDegrees()
            );
            
        } else {
            
            armInputSpeed = armControl.getAsDouble();
            
        }
        
        arm.setArmSpeed(armSpeedTransform.apply(armInputSpeed));
        
    }
    
    @Override
    public void end (boolean interrupted) {
        arm.stop();
    }
    
}

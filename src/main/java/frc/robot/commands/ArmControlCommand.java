package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Claw.ClawMovement;

public class ArmControlCommand extends CommandBase {
    
    private final Arm arm;
    private final Claw claw;
    private final DoubleSupplier armControl;
    private final BooleanSupplier armToLow, armToMid, armToHigh, grabControl, releaseControl;
    
    private Optional<Rotation2d> armSetPosition = Optional.empty();
    
    private static Transform getBoundSpeedLimitTransform (DoubleSupplier limitOffsetSupplier, double limitApplicationRange, boolean isPositiveLimit) {
        Transform limitOffsetToSpeedProp =
            ((Transform)Math::abs)
            .then(offset -> offset / limitApplicationRange)
            .then(Transform.clamp(0, 1));
        
        return (speed) -> ((speed > 0) == isPositiveLimit)
            ? limitOffsetToSpeedProp.apply(limitOffsetSupplier.getAsDouble()) * speed
            : speed;
    }
    
    private final Transform armControlInputTransform =
        InputTransform.getInputTransform(InputTransform.THREE_HALVES_CURVE, 0.1);
    
    private final Transform boundSpeedLimitsTransform;
    
    public ArmControlCommand (
        Arm arm,
        Claw claw,
        DoubleSupplier armControl,
        BooleanSupplier armToLow,
        BooleanSupplier armToMid,
        BooleanSupplier armToHigh,
        
        BooleanSupplier grabControl,
        BooleanSupplier releaseControl
    ) {
        this.arm = arm;
        this.claw = claw;
        this.armControl = armControl;
        this.armToLow = armToLow;
        this.armToMid = armToMid;
        this.armToHigh = armToHigh;
        
        this.grabControl = grabControl;
        this.releaseControl = releaseControl;
        
        boundSpeedLimitsTransform =
            getBoundSpeedLimitTransform(() -> arm.getArmRotation().getDegrees() - Arm.ARM_MAX_ANGLE_DEGREES, 30, true)
            .then(getBoundSpeedLimitTransform(() -> arm.getArmRotation().getDegrees() - Arm.ARM_MIN_ANGLE_DEGREES, 40, false));
        
        addRequirements(arm, claw);
    }
    
    @Override
    public void initialize () {
        arm.stop();
    }
    
    @Override
    public void execute () {
        
        // Set claw movement
        if (!claw.hasBeenHomed()) {
            claw.runClawHomingSequence();
        } else if (grabControl.getAsBoolean()) {
            claw.operateClaw(ClawMovement.GRAB);
        } else if (releaseControl.getAsBoolean()) {
            claw.operateClaw(ClawMovement.RELEASE);
        } else {
            claw.operateClaw(ClawMovement.NONE);
        }
        
        double armControlInput = armControlInputTransform.apply(armControl.getAsDouble());
        
        // Set arm movement
        if (armControlInput != 0) {
            armSetPosition = Optional.empty();
        } else if (armToLow.getAsBoolean()) {
            armSetPosition = Optional.of(ArmPosition.LOW.rotation);
        } else if (armToMid.getAsBoolean()) {
            armSetPosition = Optional.of(ArmPosition.MIDDLE.rotation);
        } else if (armToHigh.getAsBoolean()) {
            armSetPosition = Optional.of(ArmPosition.HIGH.rotation);
        }
        
        double armInputSpeed = 0;
        if (armSetPosition.isPresent()) {
            armInputSpeed = arm.getSpeedToMoveToRotation(armSetPosition.get());
        } else {
            armInputSpeed = boundSpeedLimitsTransform.apply(armControlInput);
        }
        
        arm.setArmSpeed(armInputSpeed);
        
    }
    
    @Override
    public void end (boolean interrupted) {
        arm.stop();
    }
    
}

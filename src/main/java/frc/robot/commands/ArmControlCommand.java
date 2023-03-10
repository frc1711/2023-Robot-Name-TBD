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
    private final BooleanSupplier armToLow, armToMid, armToHigh, grabControl, releaseControl, stabilizeArmControl;
    
    private Optional<Rotation2d> armSetPosition = Optional.empty();
    
    private final Transform armSpeedTransform =
        InputTransform.getInputTransform(InputTransform.THREE_HALVES_CURVE, 0.1);
    
    
    public ArmControlCommand (
        Arm arm,
        Claw claw,
        DoubleSupplier armControl,
        BooleanSupplier armToLow,
        BooleanSupplier armToMid,
        BooleanSupplier armToHigh,
        
        BooleanSupplier grabControl,
        BooleanSupplier releaseControl,
        
        BooleanSupplier stablizeArmControl
    ) {
        this.arm = arm;
        this.claw = claw;
        this.armControl = armControl;
        this.armToLow = armToLow;
        this.armToMid = armToMid;
        this.armToHigh = armToHigh;
        
        this.grabControl = grabControl;
        this.releaseControl = releaseControl;
        
        this.stabilizeArmControl = stablizeArmControl;
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
        
        // Set arm movement
        if (armControl.getAsDouble() != 0) {
            armSetPosition = Optional.empty();
        } else if (armToLow.getAsBoolean()) {
            armSetPosition = Optional.of(ArmPosition.LOW.rotation);
        } else if (armToMid.getAsBoolean()) {
            armSetPosition = Optional.of(ArmPosition.MIDDLE.rotation);
        } else if (armToHigh.getAsBoolean()) {
            armSetPosition = Optional.of(ArmPosition.HIGH.rotation);
        } else if (stabilizeArmControl.getAsBoolean()) {
            armSetPosition = Optional.of(arm.getArmRotation());
        }
        
        double armInputSpeed = 0;
        if (armSetPosition.isPresent()) {
            armInputSpeed = arm.getSpeedToMoveToRotation(armSetPosition.get());
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

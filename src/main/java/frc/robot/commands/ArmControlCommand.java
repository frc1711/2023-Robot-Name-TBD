package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import claw.math.InputTransform;
import claw.math.Transform;
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
    
    private final Transform armVoltageTransform = InputTransform.getInputTransform(InputTransform.THREE_HALVES_CURVE, 0.1);
    
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
        addRequirements(arm);
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
        if (armToLow.getAsBoolean()) {
            
            arm.moveArmToPosition(ArmPosition.LOW);
            
        } else if (armToMid.getAsBoolean()) {
            
            arm.moveArmToPosition(ArmPosition.MIDDLE);
            
        } else if (armToHigh.getAsBoolean()) {
            
            arm.moveArmToPosition(ArmPosition.HIGH);
            
        } else {
            
            arm.setArmSpeed(armVoltageTransform.apply(armControl.getAsDouble()));
            
        }
        
    }
    
    @Override
    public void end (boolean interrupted) {
        arm.stop();
    }
    
}

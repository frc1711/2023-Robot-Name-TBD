package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import claw.math.InputTransform;
import claw.math.Transform;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ClawMovement;

public class ArmControlCommand extends CommandBase {
    
    private final Arm arm;
    private final DoubleSupplier armControl;
    private final BooleanSupplier grabControl, releaseControl;
    
    private final Transform armVoltageTransform =
        InputTransform.getInputTransform(InputTransform.THREE_HALVES_CURVE, 0.1)
        .then((Transform)(x -> 7*x));
    
    public ArmControlCommand (Arm arm, DoubleSupplier armControl, BooleanSupplier grabControl, BooleanSupplier releaseControl) {
        this.arm = arm;
        this.armControl = armControl;
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
        
        if (!arm.hasClawBeenHomed()) {
            arm.runClawHomingSequence();
        } else if (grabControl.getAsBoolean()) {
            arm.operateClaw(ClawMovement.GRAB);
        } else if (releaseControl.getAsBoolean()) {
            arm.operateClaw(ClawMovement.RELEASE);
        } else {
            arm.operateClaw(ClawMovement.NONE);
        }
        
        arm.setArmVoltage(armVoltageTransform.apply(armControl.getAsDouble()));
        
    }
    
    @Override
    public void end (boolean interrupted) {
        arm.stop();
    }
    
}

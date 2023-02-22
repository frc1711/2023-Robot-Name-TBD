// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ClawMovement;

public class CentralCommand extends CommandBase {
    private final Arm arm;
    private final Conveyor conveyor;
    private final Intake intake;
    private final BooleanSupplier
        armController,
        intakeController,
        topBarController,
        lowerBarController,
        operateClaw,
        operateConveyor,
        reverseMode,
        safetyBrake,
        slowMode;
    
    public CentralCommand(
            Arm arm,
            Conveyor conveyor,
            Intake intake,
            BooleanSupplier armController,
            BooleanSupplier intakeController,
            BooleanSupplier topBarController,
            BooleanSupplier lowerBarController,
            BooleanSupplier operateClaw,
            BooleanSupplier operateConveyor,
            BooleanSupplier reverseMode,
            BooleanSupplier safetyBrake,
            BooleanSupplier slowMode) {
        this.arm = arm;
        this.conveyor = conveyor;
        this.intake = intake;
        this.armController = armController;
        this.operateConveyor = operateConveyor;
        this.intakeController = intakeController;
        this.topBarController = topBarController;
        this.lowerBarController = lowerBarController;
        this.operateClaw = operateClaw;
        this.reverseMode = reverseMode;
        this.safetyBrake = safetyBrake;
        this.slowMode = slowMode;
        addRequirements(arm, conveyor, intake);
    }
    
    @Override
    public void initialize() {
        arm.stop();
        conveyor.stop();
        intake.stopAll();
    }
    
    @Override
    public void execute() {
        int r = reverseMode.getAsBoolean() ? -1 : 1;
        double s = slowMode.getAsBoolean() ? .5 : 1;
        
        if (safetyBrake.getAsBoolean()) {
            arm.stop();
            conveyor.stop();
            intake.stopAll();
        } else {
            arm.setArmSpeed(armController.getAsBoolean() ? r * s : 0);
            arm.operateClaw(ClawMovement.NONE);
            conveyor.setSpeed(operateConveyor.getAsBoolean() ? r * s : 0);
            intake.setLowerBarSpeed(lowerBarController.getAsBoolean() ? r * s : 0);
            intake.setTopBarSpeed(topBarController.getAsBoolean() ? r * s : 0);
            intake.operateArmBound(intakeController.getAsBoolean() ? r * s : 0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.stop();
        conveyor.stop();
        intake.stopAll();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

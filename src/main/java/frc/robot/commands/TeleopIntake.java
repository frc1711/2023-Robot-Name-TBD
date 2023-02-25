// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Conveyor.ConveyorMode;
import frc.robot.subsystems.Intake.IntakeEngagement;
import frc.robot.subsystems.Intake.IntakeMode;

public class TeleopIntake extends CommandBase {
    
    private final Conveyor conveyor;
    private final Intake intake;
    private final BooleanSupplier intakeControl;
    private final BooleanSupplier reverseControl;
    
    private final Debouncer runConveyorDebouncer = new Debouncer(2, DebounceType.kFalling);
    
    public TeleopIntake(
        Conveyor conveyor,
        Intake intake,
        BooleanSupplier intakeControl,
        BooleanSupplier reverseControl
    ) {
        this.conveyor = conveyor;
        this.intake = intake;
        this.intakeControl = intakeControl;
        this.reverseControl = reverseControl;
        addRequirements(conveyor, intake);
    }
    
    @Override
    public void initialize() {
        conveyor.setMode(ConveyorMode.STOP);
        intake.stop();
    }
    
    @Override
    public void execute() {
        
        if (intakeControl.getAsBoolean()) {
            
            // Engage the intake
            intake.setIntakeEngagement(IntakeEngagement.ENGAGE);
            
            // Set the intake to run if it is mostly deployed
            if (intake.getEngagementPosition() > 0.7) {
                intake.setIntakeMode(IntakeMode.FORWARD);
            } else {
                intake.setIntakeMode(IntakeMode.STOP);
            }
            
        } else {
            
            // Disengage the intake
            intake.setIntakeEngagement(IntakeEngagement.DISENGAGE);
            
            // Set the intake to run in reverse if it is deployed far enough
            double intakePos = intake.getEngagementPosition();
            if (intakePos > 0.8) {
                intake.setIntakeMode(IntakeMode.FORWARD);
            } else if (intakePos > 0.3) {
                intake.setIntakeMode(IntakeMode.REVERSE);
            } else {
                intake.setIntakeMode(IntakeMode.STOP);
            }
            
        }
        
        conveyor.setMode(runConveyorDebouncer.calculate(intakeControl.getAsBoolean()) ? ConveyorMode.FORWARD : ConveyorMode.STOP);
        
    }
    
    @Override
    public void end (boolean interrupted) {
        conveyor.setMode(ConveyorMode.STOP);
        intake.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}

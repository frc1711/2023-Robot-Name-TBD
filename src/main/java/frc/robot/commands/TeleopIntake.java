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
import frc.robot.subsystems.Intake.IntakeSpeedMode;

public class TeleopIntake extends CommandBase {
    
    private final Conveyor conveyor;
    private final Intake intake;
    private final BooleanSupplier intakeCubeControl, intakeConeControl;
    
    private final Debouncer runConveyorDebouncer = new Debouncer(2, DebounceType.kFalling);
    private IntakeRunType lastIntakeRunType = IntakeRunType.NONE;
    
    public TeleopIntake(
        Conveyor conveyor,
        Intake intake,
        BooleanSupplier intakeCubeControl,
        BooleanSupplier intakeConeControl
    ) {
        this.conveyor = conveyor;
        this.intake = intake;
        this.intakeCubeControl = intakeCubeControl;
        this.intakeConeControl = intakeConeControl;
        addRequirements(conveyor, intake);
    }
    
    @Override
    public void initialize() {
        conveyor.setMode(ConveyorMode.STOP);
        intake.stop();
    }
    
    private enum IntakeRunType {
        CONE (IntakeSpeedMode.CONE, IntakeSpeedMode.CONE_REVERSE),
        CUBE (IntakeSpeedMode.CUBE, IntakeSpeedMode.CUBE_REVERSE),
        NONE (IntakeSpeedMode.STOP, IntakeSpeedMode.STOP);
        
        private final IntakeSpeedMode forwardMode, reverseMode;
        private IntakeRunType (IntakeSpeedMode forwardMode, IntakeSpeedMode reverseMode) {
            this.forwardMode = forwardMode;
            this.reverseMode = reverseMode;
        }
        
        private void runOnIntake (Intake intake) {
            final double engagementPosition = intake.getEngagementPosition();
            if (engagementPosition > 0.75) {
                intake.setIntakeSpeedMode(forwardMode);
            } else if (engagementPosition > 0.4) {
                intake.setIntakeSpeedMode(reverseMode);
            } else {
                intake.setIntakeSpeedMode(IntakeSpeedMode.STOP);
            }
        }
    }
    
    private IntakeRunType getCurrentRunType () {
        if (intakeConeControl.getAsBoolean()) {
            return IntakeRunType.CONE;
        } else if (intakeCubeControl.getAsBoolean()) {
            return IntakeRunType.CUBE;
        } else {
            return IntakeRunType.NONE;
        }
    }
    
    @Override
    public void execute() {
        
        // Set the last intake run type if we're actively being commanded to running the intake
        IntakeRunType runType = getCurrentRunType();
        if (runType != IntakeRunType.NONE) {
            intake.setIntakeEngagement(IntakeEngagement.ENGAGE);
            lastIntakeRunType = runType;
        } else {
            intake.setIntakeEngagement(IntakeEngagement.DISENGAGE);
        }
        
        // Run the conveyor according to a debouncer, allowing for running for a period
        // of time after the intake is stopped
        conveyor.setMode(
            runConveyorDebouncer.calculate(runType != IntakeRunType.NONE)
                ? ConveyorMode.FORWARD
                : ConveyorMode.STOP
        );
        
        // Run the last intake type so that while disengaging the intake continues to run
        lastIntakeRunType.runOnIntake(intake);
        
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

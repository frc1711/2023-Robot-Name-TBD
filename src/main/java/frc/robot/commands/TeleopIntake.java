// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.IntakeEngagementCommand.IntakeEngagement;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class TeleopIntake extends CommandBase {
    
    private final Conveyor conveyor;
    private final Intake intake;
    private final DoubleSupplier intakeControl;
    private final BooleanSupplier reverseConveyor;
    
    // private final Debouncer intakeDebouncer = new Debouncer(2, DebounceType.kFalling);
    
    public TeleopIntake(
        Conveyor conveyor,
        Intake intake,
        DoubleSupplier intakeControl,
        BooleanSupplier reverseConveyor
    ) {
        this.conveyor = conveyor;
        this.intake = intake;
        this.intakeControl = intakeControl;
        this.reverseConveyor = reverseConveyor;
        addRequirements(conveyor, intake);
    }
    
    @Override
    public void initialize() {
        conveyor.stop();
        intake.stop();
    }
    
    @Override
    public void execute() {
        double value = intakeControl.getAsDouble();
        
        // int conveyorForwardSpeed = reverseConveyor.getAsBoolean() ? -3 : 3;
        // if (value > 0.5) {
        //     intake.setIntakeMode(IntakeMode.FORWARD);
        //     conveyor.setSpeed(conveyorForwardSpeed);
        // } else if (value < -0.5) {
        //     intake.setIntakeMode(IntakeMode.REVERSE);
        //     conveyor.setSpeed(-conveyorForwardSpeed);
        // } else {
        //     intake.setIntakeMode(IntakeMode.STOP);
        //     conveyor.stop();
        // }
        // intake.setEngagementVoltage(0);
        
        if (value > 0.8 && !intake.isUpperPressed()) {
            new IntakeEngagementCommand(intake, IntakeEngagement.DISENGAGE)
                .until(() -> intakeControl.getAsDouble() < 0.6)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .schedule();
        } else if (value < -0.8 && !intake.isLowerPressed()) {
            new IntakeEngagementCommand(intake, IntakeEngagement.ENGAGE)
                .until(() -> intakeControl.getAsDouble() > -0.6)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .schedule();
        } else intake.stop();
        
    }
    
    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        intake.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}

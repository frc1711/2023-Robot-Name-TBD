// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class TeleopIntake extends CommandBase {
  
  private final Conveyor conveyor;
  private final BooleanSupplier runIntake;
  
  private final Debouncer intakeDebouncer = new Debouncer(2);

  public TeleopIntake(
      Conveyor conveyor,
      BooleanSupplier runIntake
  ) {
    this.conveyor = conveyor;
    this.runIntake = runIntake;
  }

  
  @Override
  public void initialize() {
    conveyor.stop();
  }

  
  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}

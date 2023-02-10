// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class CentralCommand extends CommandBase {

  private Arm arm;
  private Conveyor conveyor;
  private Intake intake;
  private DoubleSupplier armController, intakeController, topBarController, lowerBarController;
  private BooleanSupplier operateClaw, operateConveyor, reverseMode, safetyBrake, slowMode;

  private InputCurve 
      ARM_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(.014),
      INTAKE_CURVE = InputCurve.THREE_HALVES_CURVE.withDeadband(0.14);
      

  public CentralCommand(Arm arm, Conveyor conveyor, Intake intake, DoubleSupplier armController, DoubleSupplier intakeController, DoubleSupplier topBarController, DoubleSupplier lowerBarController, BooleanSupplier operateClaw, BooleanSupplier operateConveyor, BooleanSupplier reverseMode, BooleanSupplier safetyBrake, BooleanSupplier slowMode) {
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
  }

  @Override
  public void initialize() {
    arm.stopAll();
    conveyor.stop();
    intake.stopAll();
  }

  private double reverseMultiplier = 1;
  private double slowModeMultiplier = 1;
  @Override
  public void execute() {
    if (safetyBrake.getAsBoolean()) {
      arm.stopAll();
      conveyor.stop();
      intake.stopAll();
    }
    else if (reverseMode.getAsBoolean()) {
      reverseMultiplier = -1;
    }
    else if (slowMode.getAsBoolean()) {
      slowModeMultiplier = .5;
    }
    else if (operateClaw.getAsBoolean()) {
      arm.operateClaw(.5 * reverseMultiplier);
    }
    else if (operateConveyor.getAsBoolean()) {
      conveyor.set(.5 * reverseMultiplier * slowModeMultiplier);
    }
    else {
      arm.setArmSpeed(ARM_CURVE.getValue(armController.getAsDouble() * reverseMultiplier * slowModeMultiplier));
      intake.operateArmBound(INTAKE_CURVE.getValue(intakeController.getAsDouble() * reverseMultiplier * slowModeMultiplier));
      intake.setLowerBarSpeed(INTAKE_CURVE.getValue(lowerBarController.getAsDouble() * reverseMultiplier * slowModeMultiplier));
      intake.setTopBarSpeed(INTAKE_CURVE.getValue(topBarController.getAsDouble() * reverseMultiplier * slowModeMultiplier));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

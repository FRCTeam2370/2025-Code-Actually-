// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorVoltage extends Command {
  ElevatorSubsystem mElevatorSubsystem;
  Supplier<Double> voltage;
  /** Creates a new ElevatorVoltage. */
  public ElevatorVoltage(ElevatorSubsystem mElevatorSubsystem, Supplier<Double> voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mElevatorSubsystem = mElevatorSubsystem;
    this.voltage = voltage;
    addRequirements(mElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double volt = voltage.get();
    ElevatorSubsystem.setElevatorVolt(volt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

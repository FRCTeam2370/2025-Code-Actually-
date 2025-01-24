// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualElevatorControl extends Command {
  ElevatorSubsystem mElevatorSubsystem;
  Supplier<Double> joystickVal;
  double pos = 0.1;
  /** Creates a new ManualElevatorControl. */
  public ManualElevatorControl(ElevatorSubsystem mElevatorSubsystem, Supplier<Double> joystickVal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mElevatorSubsystem = mElevatorSubsystem;
    this.joystickVal = joystickVal;
    addRequirements(mElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(joystickVal.get() > 0.5 && pos < 4.8){
      pos += 0.01;
    }else if(joystickVal.get() < -0.5 && pos > 0.1){
      pos -= 0.01;
    }

    ElevatorSubsystem.setElevatorPos(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.setElevatorPos(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

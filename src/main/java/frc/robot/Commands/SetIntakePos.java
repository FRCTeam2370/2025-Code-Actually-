// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakePos extends Command {
  IntakeSubsystem mIntakeSubsystem;
  double position;
  /** Creates a new SetIntakePos. */
  public SetIntakePos(IntakeSubsystem mIntakeSubsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mIntakeSubsystem = mIntakeSubsystem;
    this.position = position;
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.stopShoulder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.setShoulderPos(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.stopShoulder();
    System.out.println("Exited Shoulder Command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(IntakeSubsystem.getShoulderPos() == position){
      return true;
    }else{
      return false;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ElevatorControl;
import frc.robot.Commands.ElevatorVoltage;
import frc.robot.Commands.SetIntakePos;
import frc.robot.Commands.SetManipulator;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class RobotContainer {

  //Controller Instantiations
  public static final CommandXboxController driver = new CommandXboxController(0);

  //Subsystem Instantiations
  private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private static ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //BUTTON BINDINGS YAY!!!
    driver.y().onTrue(new ElevatorControl(mElevatorSubsystem, 3.25));//runs the Elevator at a constant percent speed -1.0 to 1.0 vals
    driver.a().onTrue(new ElevatorControl(mElevatorSubsystem, 2));//runs the Elevator at a constant percent speed -1.0 to 1.0 vals
    driver.leftBumper().onTrue(new ElevatorControl(mElevatorSubsystem, 0.1));
    driver.start().onTrue(new ElevatorControl(mElevatorSubsystem, 4.85));

    driver.b().onTrue(new SetIntakePos(mIntakeSubsystem, 15));
    driver.x().onTrue(new SetIntakePos(mIntakeSubsystem, 10));

    //driver.rightBumper().toggleOnTrue(new ElevatorVoltage(mElevatorSubsystem, ()-> driver.getRawAxis(3)));

    driver.rightBumper().whileTrue(new SetManipulator(manipulatorSubsystem, 1));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

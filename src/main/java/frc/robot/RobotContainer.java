// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.SetManipulator;
import frc.robot.Commands.ElevatorCommands.ElevatorControl;
import frc.robot.Commands.ElevatorCommands.IdleElevator;
import frc.robot.Commands.ElevatorCommands.ManualElevatorControl;
import frc.robot.Commands.ElevatorCommands.StowElevator;
import frc.robot.Commands.IntakeCommands.RunIntakeRollers;
import frc.robot.Commands.IntakeCommands.SetIntakePos;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private static PIDController drivePID = new PIDController(0.2, 0, 0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController driver = joystick;
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
    private static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private static ManipulatorSubsystem mManipulatorSubsystem = new ManipulatorSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        //BUTTON BINDINGS YAY!!!
        driver.leftBumper().onTrue(new ElevatorControl(mElevatorSubsystem, 2.65));//runs the Elevator at a constant percent speed -1.0 to 1.0 vals
        driver.y().onTrue(new ElevatorControl(mElevatorSubsystem, 1.3));//runs the Elevator at a constant percent speed -1.0 to 1.0 vals
        driver.a().onTrue(new StowElevator(mElevatorSubsystem));
        driver.start().onTrue(new ElevatorControl(mElevatorSubsystem, 4.85));

        driver.b().onTrue(new SetIntakePos(mIntakeSubsystem, 3.85));
        driver.x().onTrue(new SetIntakePos(mIntakeSubsystem, 14.5));

        driver.leftTrigger().whileTrue(new RunIntakeRollers(mIntakeSubsystem, 0.5));
        driver.rightTrigger().whileTrue(new RunIntakeRollers(mIntakeSubsystem, -1));

        //driver.rightBumper().toggleOnTrue(new ElevatorVoltage(mElevatorSubsystem, ()-> driver.getRawAxis(3)));

        driver.rightBumper().whileTrue(new SetManipulator(mManipulatorSubsystem, 1));

        operator.a().toggleOnTrue(new ManualElevatorControl(mElevatorSubsystem, ()-> operator.getRightY()));
 
        operator.x().onTrue(new IdleElevator(mElevatorSubsystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

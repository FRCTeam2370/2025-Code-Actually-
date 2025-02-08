// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ClimberCommands.ControlClimberManual;
import frc.robot.Commands.ClimberCommands.SetClimberPos;
import frc.robot.Commands.ElevatorCommands.ElevatorControl;
import frc.robot.Commands.ElevatorCommands.IdleElevator;
import frc.robot.Commands.ElevatorCommands.ManualElevatorControl;
import frc.robot.Commands.ElevatorCommands.StowElevator;
import frc.robot.Commands.IntakeCommands.RunIntakeRollers;
import frc.robot.Commands.IntakeCommands.SetIntakePos;
import frc.robot.Commands.ManipulatorCommands.SetManipulator;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private static SlewRateLimiter xLimiter = new SlewRateLimiter(6);
    private static SlewRateLimiter yLimiter = new SlewRateLimiter(6);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController driver = joystick;
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
    private static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private static ManipulatorSubsystem mManipulatorSubsystem = new ManipulatorSubsystem();
    private static LimelightSubsystem mLimelightSubsystem = new LimelightSubsystem();
    private static ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();

    public static final PIDController limelightPID = new PIDController(0.55, 0, 0.005);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed * 0.5)) // Drive forward with negative Y (forward)
                    .withVelocityY(xLimiter.calculate(-joystick.getLeftX() * MaxSpeed * 0.5)) // Drive left with negative X (left)
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
        driver.leftBumper().onTrue(new ElevatorControl(mElevatorSubsystem, 2.65));//setpoints for the elevator: L3
        driver.y().onTrue(new ElevatorControl(mElevatorSubsystem, 1.3));//setpoints for the elevator: L2
        driver.a().onTrue(new StowElevator(mElevatorSubsystem));
        driver.start().onTrue(new ElevatorControl(mElevatorSubsystem, 4.85));//setpoints for the elevator: L4

        // driver.b().onTrue(new SetIntakePos(mIntakeSubsystem, 3.85));
        // driver.x().onTrue(new SetIntakePos(mIntakeSubsystem, 14.5));

        // driver.leftTrigger().whileTrue(new RunIntakeRollers(mIntakeSubsystem, 0.5));
        // driver.rightTrigger().whileTrue(new RunIntakeRollers(mIntakeSubsystem, -1));

        driver.rightTrigger().whileTrue(new ControlClimberManual(mClimberSubsystem, 0.25));
        driver.leftTrigger().whileTrue(new ControlClimberManual(mClimberSubsystem, -0.25));

        driver.b().toggleOnTrue((drivetrain.applyRequest(() -> driveRobot.withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed)).withVelocityY(limelightPID.calculate(LimelightSubsystem.getTx())))));
        //driver.rightBumper().toggleOnTrue(new ElevatorVoltage(mElevatorSubsystem, ()-> driver.getRawAxis(3)));

        driver.x().onTrue(new SetClimberPos(1, mClimberSubsystem));

        driver.rightBumper().whileTrue(new SetManipulator(mManipulatorSubsystem, 1));

        operator.a().toggleOnTrue(new ManualElevatorControl(mElevatorSubsystem, ()-> operator.getRightY()));
 
        operator.x().onTrue(new IdleElevator(mElevatorSubsystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

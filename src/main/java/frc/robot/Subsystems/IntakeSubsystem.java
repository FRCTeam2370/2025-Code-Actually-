// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public static TalonFX IntakeShoulder = new TalonFX(Constants.IntakeConstants.IntakeShoulderID);
  public static TalonFX IntakeRollers = new TalonFX(Constants.IntakeConstants.IntakeRollersID);
  public static CANcoder IntakeEncoder = new CANcoder(Constants.IntakeConstants.IntakeEncoderID);

  public static TalonFXConfiguration IntakeShoulderConfig = new TalonFXConfiguration();
  public static TalonFXConfiguration IntakeRollersConfig = new TalonFXConfiguration();
  public static CANcoderConfiguration IntakeEncoderConfig = new CANcoderConfiguration();

  private static PositionDutyCycle ShoulderPosCycle = new PositionDutyCycle(0);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    configIntake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShoulderPos", getShoulderPos());
    SmartDashboard.putNumber("ShoulderEror", IntakeShoulder.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("IntakeEncoder", IntakeEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public static void setShoulderPos(double position){
    IntakeShoulder.setControl(ShoulderPosCycle.withPosition(position));
  }

  public static void stopShoulder(){
    IntakeShoulder.set(0);
  }

  public static double getShoulderPos(){
    return IntakeShoulder.getPosition().getValueAsDouble();
  }

  public static double getCANcoderAbsPos(){
    return IntakeEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public static double CANcoderToFalcon(double CANcoderPos){
    return 0;// needs conversion code!!!
  }

  public static void configIntake(){
    IntakeShoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    IntakeRollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    IntakeShoulderConfig.Slot0.kP = 0.13;//You should probably set up some Gravity feedforward on this guy
    IntakeShoulderConfig.Slot0.kI = 0.03;
    IntakeShoulderConfig.Slot0.kD = 0;

    IntakeRollersConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//You definitely need to change these 
    IntakeShoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    IntakeShoulder.setPosition(0);
    
    ShoulderPosCycle.Slot = 0;

    IntakeShoulder.getConfigurator().apply(IntakeShoulderConfig);

    //CANcoder configuration and other stuff
    IntakeEncoderConfig.MagnetSensor.MagnetOffset = Constants.IntakeConstants.IntakeEncoderOffset;

    IntakeEncoder.getConfigurator().apply(IntakeEncoderConfig);
  }
}

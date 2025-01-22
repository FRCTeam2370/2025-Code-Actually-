// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  public static TalonFX elevatorMotor = new TalonFX(Constants.ElevatorConstants.elevatorId);
  
  public static TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();

  private static final PositionDutyCycle elevatorPosCycle = new PositionDutyCycle(0);
  private static final MotionMagicVoltage elevatorMagic = new MotionMagicVoltage(0);
  private static final VoltageOut elevatorVoltOut = new VoltageOut(0);

  public static double lastElevatorPosition; 

  //creates a public enum that can have 4 different states of the elevator to corespond to different control modes
  public static enum ElevatorState {
    GoingUp,
    GoingDown,
    Idle,
    HoldingPosition
  }

  public static ElevatorState mElevatorState; 

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    configElevator();//resets the elevator and sends the configs to the motor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position in OutputShaft Rotations", 
      KrakenToOutputShaft(elevatorMotor.getPosition().getValueAsDouble()));//sends the position of the elevator to the Dashboard
    
    SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
  }

  public static void setElevatorPos(double position){//PID control of the position of the elevator
    position = OutputShaftToKraken(position);
    elevatorMotor.setControl(elevatorPosCycle.withPosition(position));//in rotations of the motor
  }

  public static void setElevatorPosWithMagic(double position){
    position = OutputShaftToKraken(position);
    elevatorMotor.setControl(elevatorMagic.withPosition(position));
  }

  public static void setElevatorVolt(double voltage){
    elevatorMotor.setControl(elevatorVoltOut.withOutput(voltage));
  }

  public static void setElevator(double speed){
    //Percent output of the elevator
    elevatorMotor.set(speed);
  }

  public static void stopElevator(){
    //stops the elevator
    elevatorMotor.set(0);
  }

  public static void configElevator(){
    //configures all of the values for PID and eventually Motion Magic --- Slot0 holds regular PID vals
    elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //SLOT0 CONFIGURATION --- NORMAL PID CONTROL
    elevatorConfiguration.Slot0.kP = 1.5;//needs more tunning with weight and maybe Motion Magic pls
    elevatorConfiguration.Slot0.kI = 0;
    elevatorConfiguration.Slot0.kD = 0.4;

    //SLOT1 CONFIGURATION --- MOTION MAGIC CONTROL
    var slot1Config = elevatorConfiguration.Slot1;
    slot1Config.kS = 0.4;
    slot1Config.kV = 0.12;
    slot1Config.kA = 0.02;
    slot1Config.kP = 1;
    slot1Config.kI = 0;
    slot1Config.kD = 0;

    elevatorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    elevatorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

    elevatorConfiguration.MotionMagic.MotionMagicAcceleration = 400;
    elevatorConfiguration.MotionMagic.MotionMagicJerk = 4000;

    elevatorMotor.setPosition(0);

    elevatorPosCycle.Slot = 0;
    elevatorMagic.Slot = 1;

    elevatorMotor.getConfigurator().apply(elevatorConfiguration);
  }

  private static double OutputShaftToKraken(double OutputRotations){
    double krakenVal = OutputRotations * 15.4;
    return krakenVal;
  }

  private static double KrakenToOutputShaft(double KrakenRotations){
    double OutputVal = KrakenRotations / 15.4;
    return OutputVal;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  public static SparkMax manipulatorDriver = new SparkMax(Constants.ManipulatorConstants.manipulatorDriverID, MotorType.kBrushless);
  public static SparkMax manipulatorPassenger = new SparkMax(Constants.ManipulatorConstants.manipulatorPassengerID, MotorType.kBrushless);
   public static TalonFX manipulatorWrist = new TalonFX(Constants.ManipulatorConstants.manipulatorWristID);

  private static SparkMaxConfig PassengerConfig = new SparkMaxConfig();
  private static SparkMaxConfig DriverConfig = new SparkMaxConfig();
  private static TalonFXConfiguration WristConfig = new TalonFXConfiguration();

  private static final PositionDutyCycle manipulatorPosCycle = new PositionDutyCycle(0);

  /** Creates a new ManipulatorSubsystem. */
  public ManipulatorSubsystem() {
    //configManipulator();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void runManipulator(double speed){
    manipulatorDriver.set(speed);
    manipulatorPassenger.set(-speed);
  }

  public static void stopManipulator(){
    manipulatorDriver.set(0);
    manipulatorPassenger.set(0);
  }

  public static void setManipulatorPos(double position){//PID control of the position of the wrist
    position = OutputShaftToFalcon(position);
    manipulatorWrist.setControl(manipulatorPosCycle.withPosition(position));//in rotations of the motor
  }

  private static void configManipulator(){//figure out how to get one motor to follow the other
    PassengerConfig.inverted(true);
    // PassengerConfig.follow(manipulatorDriver);

    manipulatorWrist.setPosition(0);

    WristConfig.Slot2.kP = 1;
    WristConfig.Slot2.kI = 0;
    WristConfig.Slot2.kD = 0.03;

    DriverConfig.inverted(true);

    manipulatorPassenger.configure(PassengerConfig, null, PersistMode.kPersistParameters);
    manipulatorDriver.configure(DriverConfig, null, PersistMode.kPersistParameters);
    manipulatorWrist.getConfigurator().apply(WristConfig);

    manipulatorWrist.setNeutralMode(NeutralModeValue.Coast);
  }


  //CONVERSIONS
  private static double FalconToOutputShaft (double FalconRotations) {
    double OutputVal = FalconRotations / 37.8;
    return OutputVal;
  }

  private static double OutputShaftToFalcon (double FalconRotations) {
    double OutputVal = FalconRotations * 37.8;
    return OutputVal;
  }
}

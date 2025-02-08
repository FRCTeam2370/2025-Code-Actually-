// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  public static TalonFX climberMotor = new TalonFX(Constants.ClimberConstants.climberMotorID);

  private static TalonFXConfiguration climberConfig = new TalonFXConfiguration();

  private static PositionDutyCycle climberPosCycle = new PositionDutyCycle(0);

  public static double climberOutputRotations;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    configClimber();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberOutputRotations = krakenToOutputShaft(climberMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Position", climberOutputRotations);

    climberMotor.setControl(climberPosCycle.withPosition(climberMotor.getPosition().getValue()));
  }

  public static void setClimberPos(double outputShaftRots){
    climberMotor.setControl(climberPosCycle.withPosition(OutputShaftToKraken(outputShaftRots)));
  }

  public static void runClimberPercent(double outputSpeed){
    climberMotor.set(outputSpeed);
  }

  public static void configClimber(){
    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
   
    climberConfig.Slot0.kP = 0.16;// seems to be pretty good
    climberConfig.Slot0.kI = 0;
    climberConfig.Slot0.kD = 0.01;

    climberMotor.getConfigurator().apply(climberConfig);

    climberMotor.setPosition(0);

    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private static double krakenToOutputShaft(double krakenRots){
    return krakenRots / 18.9629;
  }

  private static double OutputShaftToKraken(double outputRots){
    return outputRots * 18.9629;
  }
}

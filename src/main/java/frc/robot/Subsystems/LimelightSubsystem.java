// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  public static double getTx(){
    if(LimelightHelpers.getTX("limelight") > 1){
      return 1;
    }else if(LimelightHelpers.getTX("limelight") < -1){
      return -1;
    }else{
      return LimelightHelpers.getTX("limelight");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tx", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("Ty", LimelightHelpers.getTY("limelight"));
  }
}
  
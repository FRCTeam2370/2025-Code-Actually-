// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public static class ElevatorConstants {
        public static int elevatorId = 14;
    }

    public static class IntakeConstants {
        public static final int IntakeShoulderID = 15;
        public static final int IntakeRollersID = 16;
        public static final int IntakeEncoderID = 19;

        public static double IntakeEncoderOffset = 0.2;//Change this to what ever the offset is
    }

    public static class ManipulatorConstants {
        public static final int manipulatorDriverID = 17;
        public static final int manipulatorPassengerID = 18;
        public static final int manipulatorWristID = 20;
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 24;
    }
}

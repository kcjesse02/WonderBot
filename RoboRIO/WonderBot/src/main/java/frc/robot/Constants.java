/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ArmConstants {
    public static final int kMotorPort = 9;
    public static final int kPotentiometerPort = 0;

    public static final double kPotentiometerRange = 270.0;
    public static final double kPotentiometerOffset = -192.0;

    public static final double kP = 0.0258;
    public static final double kD = 0.00312;
    //public static final double kCos = 0.349;
  }

  public static final class ClawConstants {
    public static final int kServoPort = 8;

    public static final double kServoClosed = 0.0;
    public static final double kServoOpen = 1.0;

    public static final double kServoActionTime = 1.0;
  }

  public static final class DriveConstants {
    public static final int kLeftMotorPort = 7;
    public static final int kRightMotorPort = 6;

    public static final int[] kLeftEncoderPorts = new int[]{0, 1};
    public static final int[] kRightEncoderPorts = new int[]{2, 3};
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;

    public static final double kTrackWidthMeters = 0.27;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final int kEncoderCPR = 140;
    public static final double kWheelDiameterMeters = 0.09;
    public static final double kGearRatio = 72.0 / 90.0;
    public static final double kEncoderDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR) * kGearRatio;


    public static final double ksVolts = 1.98;
    public static final double kvVoltSecondsPerMeter = 6.11;
    public static final double kaVoltSecondsSquaredPerMeter = -0.021;
    public static final SimpleMotorFeedforward kFeedforward
        = new SimpleMotorFeedforward(ksVolts,
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter);

    public static final double kPDriveVel = 0.998;
  }
}

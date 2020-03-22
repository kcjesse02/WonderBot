/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final SpeedController leftMotor;
  private final SpeedController rightMotor;

  private final DifferentialDrive driveBase;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;
  private final Gyro gyro;

  private final DifferentialDriveOdometry odometry;

  /**
   * Creates a new Drive.
   */
  public DriveSubsystem() {
    leftMotor = new PWMTalonSRX(Constants.DriveConstants.kLeftMotorPort);
    rightMotor = new PWMTalonSRX(Constants.DriveConstants.kRightMotorPort);
    rightMotor.setInverted(true);
    driveBase = new DifferentialDrive(leftMotor, rightMotor);
    driveBase.setRightSideInverted(false);

    leftEncoder = new Encoder(Constants.DriveConstants.kLeftEncoderPorts[0], Constants.DriveConstants.kLeftEncoderPorts[1], true);
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

    rightEncoder = new Encoder(Constants.DriveConstants.kRightEncoderPorts[0], Constants.DriveConstants.kRightEncoderPorts[1]);
    rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

    gyro = new ADXRS450_Gyro();

    odometry = new DifferentialDriveOdometry(getGyroAngle());
  }

  /**
   * Drive the robot like a tank.
   */
  public void drive(double xSpeed, double zRotation, boolean isQuickTurn) {
    driveBase.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.set(leftVolts / RobotController.getBatteryVoltage());
    rightMotor.set(rightVolts / RobotController.getBatteryVoltage());
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  @Override
  public void periodic() {
    odometry.update(getGyroAngle(), leftEncoder.getDistance(), rightEncoder.getDistance());

    SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
    SmartDashboard.putNumber("Right Counts", rightEncoder.getRaw());
    SmartDashboard.putString("Position", odometry.getPoseMeters().toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the gyro angle of the robot.
   */
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-getHeading());
  }

  /**
   * Returns the gyro angle of the robot.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  private double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
}

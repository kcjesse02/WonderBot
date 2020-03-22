/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive tank style.
 */
public class FollowDriveTrajectoryCommand extends RamseteCommand {
  public FollowDriveTrajectoryCommand(DriveSubsystem drive, Trajectory trajectory) {
    super(
      trajectory,
      drive::getPose,
      new RamseteController(2, 0.7),
      Constants.DriveConstants.kFeedforward,
      Constants.DriveConstants.kDriveKinematics,
      drive::getWheelSpeeds,
      new PIDController(0, 0, 0),
      new PIDController(0, 0, 0),
      // RamseteCommand passes volts to the callback
      drive::tankDriveVolts,
      drive
    );
  }
}

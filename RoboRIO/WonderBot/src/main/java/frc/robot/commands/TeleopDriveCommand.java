/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive tank style.
 */
public class TeleopDriveCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final BooleanSupplier isQuickTurn;

  /**
   * Creates a new TankDrive command.
   *
   * @param drivetrain The drivetrain subsystem to drive
   */
  public TeleopDriveCommand(DriveSubsystem drive, DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier isQuickTurn) {
    this.drive = drive;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.isQuickTurn = isQuickTurn;

    addRequirements(drive);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    drive.drive(xSpeed.getAsDouble(), zRotation.getAsDouble(), isQuickTurn.getAsBoolean());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, false);
  }
}

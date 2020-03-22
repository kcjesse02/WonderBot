/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private final Servo servo;

  /**
   * Create a new claw subsystem.
   */
  public ClawSubsystem() {
    servo = new Servo(Constants.ClawConstants.kServoPort);
  }

  /**
   * Set the claw motor to move in the open direction.
   */
  public void open() {
    servo.set(Constants.ClawConstants.kServoOpen);
  }

  /**
   * Set the claw motor to move in the close direction.
   */
  public void close() {
    servo.set(Constants.ClawConstants.kServoClosed);
  }
}

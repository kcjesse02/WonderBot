/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

/**
 * Opens the claw for one second. Real robots should use sensors, stalling motors is BAD!
 */
public class CloseClawCommand extends WaitCommand {
  private final ClawSubsystem claw;

  /**
   * Creates a new OpenClaw command.
   *
   * @param claw The claw to use
   */
  public CloseClawCommand(ClawSubsystem claw) {
    super(Constants.ClawConstants.kServoActionTime);

    this.claw = claw;

    addRequirements(claw);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    super.initialize();
    claw.close();
  }
}

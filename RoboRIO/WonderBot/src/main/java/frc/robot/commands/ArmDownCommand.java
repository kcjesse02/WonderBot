/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDownCommand extends CommandBase {
  private final ArmSubsystem arm;

  public ArmDownCommand(ArmSubsystem arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setSetpoint(-30.0);
  }

  @Override
  public boolean isFinished() {
    return arm.atSetpoint();
  }
}

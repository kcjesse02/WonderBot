/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ArmSubsystem extends PIDSubsystem {
  private final SpeedController motor;
  private final Potentiometer potentiometer;

  /**
   * Creates a new Drive.
   */
  public ArmSubsystem() {
    super(new PIDController(Constants.ArmConstants.kP, 0.0, Constants.ArmConstants.kD));
    m_controller.setTolerance(2.0);

    motor = new VictorSP(Constants.ArmConstants.kMotorPort);
    potentiometer = new AnalogPotentiometer(Constants.ArmConstants.kPotentiometerPort,
        Constants.ArmConstants.kPotentiometerRange, Constants.ArmConstants.kPotentiometerOffset);

    SendableRegistry.addChild(this, motor);
    SendableRegistry.addChild(this, potentiometer);

    enable();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Arm Angle", getMeasurement());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    motor.set(output);
  }

  @Override
  public double getMeasurement() {
    return potentiometer.get();
  }
}

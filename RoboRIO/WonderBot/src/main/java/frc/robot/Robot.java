/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.OpenClawCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.System;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private final ClawSubsystem claw = new ClawSubsystem();

  private RobotContainer m_robotContainer;
  public NetworkTable table;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("wondertable");
    NetworkTableEntry ent = table.getEntry("command");
    NetworkTableEntry data = table.getEntry("data");
    ent.setNumber(0);
    data.setNumber(0);
    
    /*
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setResolution(640, 480);
    cam.setExposureManual(1);
    cam.setBrightness(1);
    */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //CommandScheduler.schedule(NetworkTableReader());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    NetworkTableEntry ent = table.getEntry("command");
    int command = ent.getNumber(0).intValue();
    if(command == 1){
      DriveToDistance back = new DriveToDistance(RobotContainer.drive, new Pose2d(0, 0.5, new Rotation2d(0)));
      back.schedule();
      ent.setNumber(0);
    }
    if(command == 2){
      DriveToDistance back = new DriveToDistance(RobotContainer.drive, new Pose2d(0, -0.5, new Rotation2d(0)));
      back.schedule();
      ent.setNumber(0);
    }
    if( command == 3){
      OpenClawCommand cl = new OpenClawCommand(RobotContainer.claw);
      cl.schedule();
      ent.setNumber(0);
    }
    if( command == 4){
      CloseClawCommand cl = new CloseClawCommand(RobotContainer.claw);
      cl.schedule();
      ent.setNumber(0);
    }
    if(command == 5){
      ArmUpCommand armUp = new ArmUpCommand(RobotContainer.arm, 50.0);
      armUp.schedule();
      ent.setNumber(0);
    }

    if(command == 6){
      ArmUpCommand armDown = new ArmUpCommand(RobotContainer.arm, 0.0);
      armDown.schedule();
      ent.setNumber(0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

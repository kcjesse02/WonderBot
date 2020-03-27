/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {
  /**
   * Creates a new DriveToDistance.
   */
  private boolean isFinished;
  DriveSubsystem drive;
  private DriveSubsystem drivetrain;
  private Pose2d targetDist;
  private Rotation2d targetRot;
  public DriveToDistance(DriveSubsystem drivetrain, Pose2d distTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = drivetrain;
    targetDist = distTarget;
    isFinished = false;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.reset();

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(targetDist.getTranslation().getX()< 0)
      drive.drive(-1.0, 0, false);
    else 
      drive.drive(1.0, 0, false);

    Pose2d updateDist = drive.getPose();//most recent robot position
    //printing out new robot pose and wheel speed
    System.out.println("updateDist: "+updateDist);
    System.out.println("wheel speed: "+drive.getWheelSpeeds());
    //checks to see if the translation part of the robot position is equal to the target translation.
    if(targetDist.getTranslation().getX()< 0){
      if((updateDist.getTranslation()).getX() <= (targetDist.getTranslation().getX())){
        System.out.println("Done!");
        isFinished = true;
      }
    else{  
      if((updateDist.getTranslation()).getX() >= (targetDist.getTranslation().getX())){
        System.out.println("Done!");
        isFinished = true;
      }
    }
  }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isFinished){
      drive.stop();//sets motor velocity to zero before ending.
    }
    return isFinished;
  }
}

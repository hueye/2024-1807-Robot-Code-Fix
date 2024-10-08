// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class RotateToSpeakerCMD extends Command {
  private DriveTrain driveTrain;
  private Vision visionSubsystem;
  private double initialHeading;
  private double desiredHeading;
  private double headingDifference;
  private double currentDifference;
  private PIDController turningPIDContoller;
  /** Creates a new RotateToSpeakerCMD. */
  public RotateToSpeakerCMD(DriveTrain driveTrain, Vision visionSubsystem) {
    this.driveTrain = driveTrain;
    this.visionSubsystem = visionSubsystem;
    turningPIDContoller = new PIDController(0.02, 0.0, 0.0006);
    turningPIDContoller.setTolerance(VisionConstants.rotationTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialHeading = driveTrain.getHeading();
    desiredHeading = driveTrain.getHeading() - visionSubsystem.getDegreesToSpeaker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDifference = driveTrain.getHeading()-desiredHeading;
    driveTrain.drive(0, 0, turningPIDContoller.calculate(currentDifference), true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, true, false);
    driveTrain.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(currentDifference)<VisionConstants.rotationTolerance;
  }
}

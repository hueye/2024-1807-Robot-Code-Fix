// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCMDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climb;

public class ClimbCMD extends Command {
  private Climb climbSubsystem;
  private CommandXboxController operatorController;
  /** Creates a new ClimbCMD. */
  public ClimbCMD(CommandXboxController operatorController, Climb climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    this.operatorController = operatorController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.setClimb(MathUtil.applyDeadband(operatorController.getLeftY(), .25));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

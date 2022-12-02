// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ButtonMoveReverse extends CommandBase {
  private final DriveSubsystem m_drive;
  Timer endTimer = new Timer();
  /** Creates a new ButtonMoveReverse. */
  public ButtonMoveReverse(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //reset timer and start
    endTimer.reset();
    endTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //move robot forward
     m_drive.drive(-0.3, 0, 0,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endTimer.get() >= 1) {
      return true;
    } else {
      return false;
    }
  }
}

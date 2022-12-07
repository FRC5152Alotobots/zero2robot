// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Group_DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Cmd_MoveForward extends CommandBase {
  //Define Subsys MecanumDrive
  private final Subsys_MecanumDrive i_MecanumDrive;
  //New timer 
  Timer m_class_endTimer__seconds = new Timer();
  //isDone vars
  private static final boolean m_k_bool_isDone = true;
  private static final boolean m_k_bool_isNotDone = false;
  //Constructor?
  public Cmd_MoveForward(Subsys_MecanumDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    i_MecanumDrive = drive;
    addRequirements(drive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //reset timer and start
    m_class_endTimer__seconds.reset();
    m_class_endTimer__seconds.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //move robot forward
    i_MecanumDrive.MecanumDrive(0.3, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_class_endTimer__seconds.get() >= 1) {
      return m_k_bool_isDone;
    } else {
      return m_k_bool_isNotDone;
    }
  }
}

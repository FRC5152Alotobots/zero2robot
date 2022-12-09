// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Group_TestFalcon;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Cmd_MoveToPosition extends CommandBase {  
  //Declare Variables 
  private final Subsys_FalconTest m_FalconSys;

  /**Constructor 
   * @param TestFalconSubsystem
   * @param isDpadUp 
   */
  public Cmd_MoveToPosition( Subsys_FalconTest FalconSys, int position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_FalconSys = FalconSys;
    addRequirements(m_FalconSys);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_FalconSys.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_FalconSys.setPosition(100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Group_TestFalcon;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Cmd_MoveWithJoystick extends CommandBase {  
  //Declare Variables 
  private final Subsys_FalconTest m_FalconSys;
  private DoubleSupplier joyValue;

  /**Constructor 
   * @param TestFalconSubsystem
   * @param Joystick_DS 
   */
  public Cmd_MoveWithJoystick( Subsys_FalconTest FalconSys, DoubleSupplier joyValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_FalconSys = FalconSys;
    addRequirements(m_FalconSys);
    this.joyValue = joyValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_FalconSys.setPrecentOutput(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_FalconSys.setPrecentOutput(joyValue.getAsDouble());
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
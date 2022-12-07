// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/** 
 * * For better comments extention (Add above package)
 * * importandt 
 * ! Deprecitated 
 * ? question 
 * TODO:
 * @param myParam[Sqr_brackets_for_multipl_Sim_elements(likemotors)] (Used to define params)
 */
package frc.robot.subsystems.Group_DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Subsys_MecanumDrive extends SubsystemBase {
/**
 * *Declare Motors 
 * @param k[Motor]Port configure to fit needs 
 * (You may need to use the phoenix tuner)
 */
  private final WPI_VictorSPX m_frontLeftMotor = new WPI_VictorSPX(DriveConstants.k_FrontLeftMotorPort);
  private final WPI_VictorSPX m_rearLeftMotor = new WPI_VictorSPX(DriveConstants.k_RearLeftMotorPort);
  private final WPI_VictorSPX m_frontRightMotor = new WPI_VictorSPX(DriveConstants.k_FrontRightMotorPort);
  private final WPI_VictorSPX m_rearRightMotor = new WPI_VictorSPX(DriveConstants.k_RearRightMotorPort);
//initialize Mecanum Drive 
  private final MecanumDrive m_MecanumDrive = new MecanumDrive(m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor);
//*Declare Encoders  
  /** 
   *  The front-left-side drive encoder @param k[Encoder]Reversed boolean if that encoder reversed or not 
   * True = isReversed Flase = notReversed
  */
  private final Encoder m_frontLeftEncoder =
      new Encoder(
          DriveConstants.k_FrontLeftEncoderPorts[0],
          DriveConstants.k_FrontLeftEncoderPorts[1],
          DriveConstants.k_FrontLeftEncoderReversed);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder =
      new Encoder(
          DriveConstants.k_RearLeftEncoderPorts[0],
          DriveConstants.k_RearLeftEncoderPorts[1],
          DriveConstants.k_RearLeftEncoderReversed);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder =
      new Encoder(
          DriveConstants.k_FrontRightEncoderPorts[0],
          DriveConstants.k_FrontRightEncoderPorts[1],
          DriveConstants.k_FrontRightEncoderReversed);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder =
      new Encoder(
          DriveConstants.k_RearRightEncoderPorts[0],
          DriveConstants.k_RearRightEncoderPorts[1],
          DriveConstants.k_RearRightEncoderReversed);
//*Declare Gyro
  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(DriveConstants.k_DriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public Subsys_MecanumDrive() {
    // Sets the distance per pulse for the encoders
    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.k_EncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(DriveConstants.k_EncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.k_EncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(DriveConstants.k_EncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeftMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new MecanumDriveWheelSpeeds(
            m_frontLeftEncoder.getRate(),
            m_rearLeftEncoder.getRate(),
            m_frontRightEncoder.getRate(),
            m_rearRightEncoder.getRate()));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void MecanumDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_MecanumDrive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_MecanumDrive.driveCartesian(ySpeed, xSpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeftMotor.setVoltage(volts.frontLeftVoltage);
    m_rearLeftMotor.setVoltage(volts.rearLeftVoltage);
    m_frontRightMotor.setVoltage(volts.frontRightVoltage);
    m_rearRightMotor.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_rearLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_rearRightEncoder.getRate());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_MecanumDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}

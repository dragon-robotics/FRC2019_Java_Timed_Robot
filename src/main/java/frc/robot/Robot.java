/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private final PWMVictorSPX m_frontRight = new PWMVictorSPX(4);
  private final PWMVictorSPX m_frontLeft = new PWMVictorSPX(3);
  SpeedControllerGroup m_front = new SpeedControllerGroup(m_frontRight, m_frontLeft);
  private final PWMVictorSPX m_rearRight = new PWMVictorSPX(2);
  private final PWMVictorSPX m_rearLeft = new PWMVictorSPX(1);
  SpeedControllerGroup m_rear = new SpeedControllerGroup(m_rearRight, m_rearLeft);
  private final DifferentialDrive m_robotDrive
    = new DifferentialDrive(m_front, m_rear);
  private final Talon Lifter = new Talon(5);
  private final Joystick m_stick = new Joystick(0);
  private final JoystickButton m_LB = new JoystickButton(m_stick, 5);
  private final JoystickButton m_RB = new JoystickButton(m_stick, 6);
  private final Joystick m_stick1 = new Joystick(1);
  private final Timer m_timer = new Timer();
  private final Timer lifter_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    double y = m_stick.getY();
    double x = m_stick.getX();
    if (m_LB.get()) {
      lifter_timer.start();
        if (lifter_timer.get() < 0.5) {
          Lifter.set(1);
        }
        else {
          Lifter.stopMotor();
        }
    } else {
      Lifter.stopMotor(); // stop Lifting motor
    }

    if (m_RB.get()) {
      lifter_timer.start();
        if (lifter_timer.get() < 0.5) {
          Lifter.set(-1);
        }
        else {
          Lifter.stopMotor();
        }
    } else {
      Lifter.stopMotor(); // stop Lifting motor
    }

    System.out.println("X: "+x+", Y: "+y);
    m_robotDrive.arcadeDrive(-m_stick.getY(Hand.kLeft), m_stick.getX(Hand.kRight));
  }




  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double y = m_stick.getY();
    double x = m_stick.getX();

    System.out.println("X: "+x+", Y: "+y);
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }
}
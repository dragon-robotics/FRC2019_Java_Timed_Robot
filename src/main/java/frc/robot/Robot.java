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

/* For TalonSRX operation */
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  /* Front Motors */
  private final PWMVictorSPX m_frontRight = new PWMVictorSPX(4);                        // Front Right Motor using PWM Victor
  private final PWMVictorSPX m_frontLeft = new PWMVictorSPX(3);                         // Front Left Motor using PWM Victor
  SpeedControllerGroup m_front = new SpeedControllerGroup(m_frontRight, m_frontLeft);   // Front Right + Front Left synchronized control
  
  /* Rear Motors */
  private final PWMVictorSPX m_rearRight = new PWMVictorSPX(2);                         // Rear Right Motor using PWM Victor
  private final PWMVictorSPX m_rearLeft = new PWMVictorSPX(1);                          // Rear Left Motor using PWM Victor
  SpeedControllerGroup m_rear = new SpeedControllerGroup(m_rearRight, m_rearLeft);      // Rear Right + Rear Left synchronized control
  
  /* Robot Drive Combination */
  private final DifferentialDrive m_robotDrive
    = new DifferentialDrive(m_front, m_rear);
  
  /* Landing Gear TalonSRX */
  private final WPI_TalonSRX m_landingGearLeft = new WPI_TalonSRX(1);     //  Left landingGear
  private final WPI_TalonSRX m_landingGearRight = new WPI_TalonSRX(2);
  SpeedControllerGroup m_landingGear 
    = new SpeedControllerGroup(m_landingGearLeft, m_landingGearRight);

  /* Robot Driver - AKA Driver 1 */
  private final Joystick j_stick_driver_1 = new Joystick(1);

  /* Robot Controller - AKA Driver 2 */
  private final Joystick j_stick_driver_2 = new Joystick(0);
  private final JoystickButton j_stick_driver_2_LB = new JoystickButton(j_stick_driver_2, 5);
  private final JoystickButton j_stick_driver_2_RB = new JoystickButton(j_stick_driver_2, 6);
      
  /* Timers */
  private final Timer m_timer = new Timer();
  private final Timer landingGear_timer = new Timer();

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

    /* When button is we want to run it for 0.5 seconds */
    /* When button is held, we want it to run indefinitely */

    boolean LB_pressed = j_stick_driver_2_LB.get();
    boolean RB_pressed = j_stick_driver_2_RB.get();
    
    if(LB_pressed || RB_pressed){

      // Reset and start the timer //
      landingGear_timer.reset();
      landingGear_timer.start();

      if(LB_pressed){
        // Set the motor to positive full power //
        m_landingGear.set(1);
      }
      else{
        // Set the motor to negative full power //
        m_landingGear.set(-1);
      }        
    }

    // If the motor is going for more than half a second, we will stop the motor and reset the timer //
    if(landingGear_timer.get() > 0.3){
      // Stop and reset the timer //
      m_landingGear.stopMotor();
      landingGear_timer.stop();
      landingGear_timer.reset();
    }
    
    /* Drive the robot with joystick control */
    m_robotDrive.arcadeDrive(-j_stick_driver_1.getY(Hand.kLeft), j_stick_driver_1.getX(Hand.kRight)); // Drive the robot
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double x = j_stick_driver_1.getX();
    double y = j_stick_driver_1.getY();

    System.out.println("X: "+x+", Y: "+y);
    m_robotDrive.arcadeDrive(j_stick_driver_1.getY(), j_stick_driver_1.getX());
  }
}
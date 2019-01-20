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

    /* 
      Landing gear operation:
        - The purpose of this operation is to allow the user to deploy/retract the landing gear on the robot for climbing L2 during the endgame
      What do we want to do:
        1. When button is clicked, we want it to run it for 0.5 seconds
          - Clicked is when the button goes from down to up in a short amount of time
        2. When button is held/pressed, we want to run it indefinitely
          - Held/Pressed is when the button goes down and stays there for a relatively medium to long period of time
      How the code works / is implemented:
        - Because the teleopPeriodic() function is repeatedly called every 20 milliseconds, we can consider all of the code inside of a function
          as if it's ran inside of a loop. Therefore, when we write the code, we want to be wary of the fact that there might be code that don't 
          need to be used until it's needed. In this case, the best way to think about this loop is a  do(action)...until(condition) loop. The 
          reason for this is because for the first case, we want to keep the landing gear motor running for some amount of time before stopping. 
          Therefore our action="keep running the landing gear motor at full power based on which button is pressed" and our condition="X amount 
          of seconds". To keep track of time, I have the timer reset and immediately start after resetting. After checking whether left or right 
          button is pressed, we set the motor power and direction accordingly. Our end condition is checking whether or not we are equal to or 
          greater than the amount of time we want the landing gear motors to run for. For the second case, because we are continually holding the
          button, we want the landing gears motor to run indefinitely. To do this, we have to reset and start the timer every single time a left
          or right button is held because we never want our timer to be above 0.3 seconds. One last thing is we also want to make sure only one
          button is pressed and when both button is pressed, we retain to prevent unknown behaviors in the code. This is done through the 
          "^" (XOR or exclusive or) operation when we check exclusively whether which button was pressed.
    */


    boolean LB_pressed = j_stick_driver_2_LB.get();   // Check whether the LB is pressed or not. Returns a boolean value (True/False)
    boolean RB_pressed = j_stick_driver_2_RB.get();   // Check whether the RB is pressed or not. Returns a boolean value (True/False) 

    // Check if either button is exclusively pressed, (^ is the XOR operator) //
    if(LB_pressed ^ RB_pressed){

      // If either button is pressed, reset and start the timer //

      // Reset and start the timer //
      landingGear_timer.reset();  // This will reset the timer back to 0 seconds
      landingGear_timer.start();  // This will start the timer counting from 0 seconds (This is because we just reset the timer)

      // Check which button is pressed and set the motor power and direction accordingly //
      if(LB_pressed){
        // If the left button is pressed //
        // Set the motor to positive full power //
        m_landingGear.set(1);   // Set the landing gear to 100% power (set range is between -1.0 and 1.0)
      }
      else{
        // If the right button is pressed //
        // Set the motor to negative full power //
        m_landingGear.set(-1);  // Set the landing gear to -100% power (set range is between -1.0 and 1.0)
      }        
    }

    // If the motor is going for more than 0.3 seconds, we will stop the motor and reset the timer //
    if(landingGear_timer.get() >= 0.3){
      // Stop and reset the timer //
      m_landingGear.stopMotor();    // Stop the motor
      landingGear_timer.stop();     // Stop the timer
      landingGear_timer.reset();    // Reset the timer
    }
    
    /* Drive the robot with joystick control. Left analog stick controls up/down. Right analog stick controls left/right */
    double leftJoyY = -j_stick_driver_1.getY(Hand.kLeft);   // This is reversed because of arcade drive implementation
    double rightJoyX = j_stick_driver_1.getX(Hand.kRight);  // Grab right analog stick X value
    m_robotDrive.arcadeDrive(leftJoyY, rightJoyX);          // Drive the robot using arcade drive
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
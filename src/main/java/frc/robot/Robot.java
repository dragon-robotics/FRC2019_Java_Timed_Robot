/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;                  // A WPI library used for joystick control
import edu.wpi.first.wpilibj.PWMVictorSPX;              // A WPI library used for PWM Victor Motor Contorller
import edu.wpi.first.wpilibj.TimedRobot;                // A WPI library used for Timed Robot base class
import edu.wpi.first.wpilibj.Timer;                     // A WPI library used for timing robot operations
import edu.wpi.first.wpilibj.buttons.JoystickButton;    // A WPI library used for joystick buttons mapping
import edu.wpi.first.wpilibj.drive.DifferentialDrive;   // A WPI library used for controlling differential drive
import edu.wpi.first.wpilibj.SpeedControllerGroup;      // A WPI library used for grouping motors for synchronized controls

/* Pneumatics Control */
import edu.wpi.first.wpilibj.Compressor;                // A WPI library used for controlling and obtaining the status of the compressor
import edu.wpi.first.wpilibj.Solenoid;                  // A WPI library used for controlling and obtaining the status of the single solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;            // A WPI library used for controlling and obtaining the status of the double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;      // Enumeration for double solenoid direction

/* For TalonSRX operation */
import com.ctre.phoenix.motorcontrol.ControlMode;       // A CTRE library used for control mode selection in the CTRE library
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;  // A CTRE library used for TalonSRX control using CAN protocol

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  /* Left Motors */
  private final PWMVictorSPX m_leftFront = new PWMVictorSPX(2);                         // Left Front Motor using PWM Victor
  private final PWMVictorSPX m_leftRear = new PWMVictorSPX(0);                          // Left Rear Motor using PWM Victor
  SpeedControllerGroup m_left = new SpeedControllerGroup(m_leftFront, m_leftRear);      // Left Front + Left Rear synchronized control
  
  /* Right Motors */
  private final PWMVictorSPX m_rightFront = new PWMVictorSPX(3);                        // Right Front Motor using PWM Victor
  private final PWMVictorSPX m_rightRear = new PWMVictorSPX(1);                         // Right Rear Motor using PWM Victor
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_rightFront, m_rightRear);   // Right Front + Right Rear synchronized control
  
  /* Robot Drive Combination */
  private final DifferentialDrive m_robotDrive
    = new DifferentialDrive(m_left, m_right);
  

  /* Cargo TalonSRX */
  private final WPI_TalonSRX m_cargo = new WPI_TalonSRX(1); // Cargo motor for controlling the cargo shooter
  
  private final WPI_TalonSRX m_lift_cargo_left = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_lift_cargo_right = new WPI_TalonSRX(4); // Cargo motor for controlling the cargo shooter
 // SpeedControllerGroup m_lift_cargo = new SpeedControllerGroup(m_lift_cargo_left, m_lift_cargo_right);

  /* Hatch TalonSRX */
  private final WPI_TalonSRX m_hatch = new WPI_TalonSRX(3); // Hatch Panel motor for controlling the elephant trunk

  /* Robot Driver - AKA Driver 1 */
  private final Joystick j_stick_driver = new Joystick(0);  // Drive joystick initialization
  private final JoystickButton j_stick_driver_LB = new JoystickButton(j_stick_driver, 5);
  private final JoystickButton j_stick_driver_RB = new JoystickButton(j_stick_driver, 6);           // Right button 
  private final JoystickButton j_stick_driver_X = new JoystickButton(j_stick_driver, 3);            // X button
  private final JoystickButton j_stick_driver_Y = new JoystickButton(j_stick_driver, 4);            // Y button
  private final JoystickButton j_stick_driver_A = new JoystickButton(j_stick_driver, 1);            // A button
  private final JoystickButton j_stick_driver_B = new JoystickButton(j_stick_driver, 2);            // B button
  private final JoystickButton j_stick_driver_Back = new JoystickButton(j_stick_driver, 7);         // Back button
  private final JoystickButton j_stick_driver_Start = new JoystickButton(j_stick_driver, 8);        // Start button
  private final JoystickButton j_stick_driver_leftStick = new JoystickButton(j_stick_driver, 9);    // Left-Stick button
  private final JoystickButton j_stick_driver_rightStick = new JoystickButton(j_stick_driver, 10);  // Right-Stick button

  /* Robot Controller - AKA Driver 2 */
  private final Joystick j_stick_control = new Joystick(1);  // Control joystick initialization
  private final JoystickButton j_stick_control_LB = new JoystickButton(j_stick_control, 5);           // Left button
  private final JoystickButton j_stick_control_RB = new JoystickButton(j_stick_control, 6);           // Right button 
  private final JoystickButton j_stick_control_X = new JoystickButton(j_stick_control, 3);            // X button
  private final JoystickButton j_stick_control_Y = new JoystickButton(j_stick_control, 4);            // Y button
  private final JoystickButton j_stick_control_A = new JoystickButton(j_stick_control, 1);            // A button
  private final JoystickButton j_stick_control_B = new JoystickButton(j_stick_control, 2);            // B button
  private final JoystickButton j_stick_control_Back = new JoystickButton(j_stick_control, 7);         // Back button
  private final JoystickButton j_stick_control_Start = new JoystickButton(j_stick_control, 8);        // Start button
  private final JoystickButton j_stick_control_rightStick = new JoystickButton(j_stick_control, 10);  // Right-Stick button

  /* Timers */
  private final Timer m_timer = new Timer();            // Generic timer
  private final Timer landingGear_timer = new Timer();  // Landing gear timer

  /* Compressor */
  private final Compressor compressor = new Compressor(0);  // Compressor code
  
  /* Double Solenoids */
  private final DoubleSolenoid doubleSolenoid_Hatch = new DoubleSolenoid(0,1);
  private final DoubleSolenoid doubleSolenoid_Front = new DoubleSolenoid(2,3);    // The right solenoid
  private final DoubleSolenoid doubleSolenoid_Back = new DoubleSolenoid(4,5);   // The left solenoid
  private final DoubleSolenoid testSolenoid_2 = new DoubleSolenoid(6,7);

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
    if (m_timer.get() < 1.0) {
      m_robotDrive.arcadeDrive(0.75, 0.0); // drive forwards half speed
    } else if (m_timer.get() < 2.6 && m_timer.get() > 1.0) {
      m_robotDrive.arcadeDrive(0, 0.5);
    } else {
      this.teleopPeriodic(); // stop robot
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
          need to be used until it's needed. In this case, the best way to think about this loop is a do(action)...until(condition) loop. The 
          reason for this is because for the first case, we want to keep the landing gear motor running for some amount of time before stopping. 
          Therefore our action="keep running the landing gear motor at full power based on which button is pressed" and our condition="X amount 
          of seconds". To keep track of time, I have the timer reset and immediately start after resetting after the left or right button is
          pressed. After checking whether left or right button is pressed, we set the motor power and direction accordingly. Our end condition 
          is checking whether or not we are equal to or greater than the amount of time we want the landing gear motors to run for. 
          
          For the second case, because we are continually holding the button, we want the landing gears motor to run indefinitely. To do this,
          we have to reset and start the timer every single time a left or right button is held because we never want our timer to be above 0.3
          seconds. One last thing is we also want to make sure only one button is pressed and when both button is pressed, we retain to prevent
          unknown behaviors in the code. This is done through the "^" (XOR or exclusive or) operation when we check exclusively whether which 
          button was pressed.
          */
    boolean LB_driver_pressed = j_stick_driver_LB.get();      
          
    boolean LB_pressed = j_stick_control_LB.get();   // Check whether the LB is pressed or not. Returns a boolean value (True/False)
    boolean RB_pressed = j_stick_control_RB.get();   // Check whether the RB is pressed or not. Returns a boolean value (True/False)
    boolean X_driver_pressed = j_stick_driver_X.get();   // Get X pressed value
    boolean Y_driver_pressed = j_stick_driver_Y.get();   // Get Y pressed value
    boolean A_driver_pressed = j_stick_driver_A.get(); // Get A pressed value
    boolean B_driver_pressed = j_stick_driver_B.get(); // Get B pressed value

    boolean X_pressed = j_stick_control_X.get(); // Get X pressed value
    boolean Y_pressed = j_stick_control_Y.get(); // Get Y pressed value
    boolean A_pressed = j_stick_control_A.get(); // Get A pressed value
    boolean B_pressed = j_stick_control_B.get(); // Get B pressed value
    boolean Start_pressed = j_stick_control_Start.get();
    boolean Back_pressed = j_stick_control_Back.get();

    boolean rightStick_pressed = j_stick_control_rightStick.get();  // Get right stick pressed value

    // Check if either button is exclusively pressed, (^ is the XOR operator) //
   /* if(X_pressed ^ Y_pressed){

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
    } */

    // If the motor is going for more than 0.3 seconds, we will stop the motor and reset the timer //
    /*if(landingGear_timer.get() >= 0.3){
      // Stop and reset the timer //
      m_landingGear.stopMotor();    // Stop the motor
      landingGear_timer.stop();     // Stop the timer
      landingGear_timer.reset();    // Reset the timer
    }*/
    double control_leftJoyX = j_stick_control.getRawAxis(1);
    // m_hatch.()
    
    /* Drive the robot with joystick control. Left analog stick controls up/down. Right analog stick controls left/right */
    double leftJoyY = j_stick_driver.getRawAxis(1);    // Grab left analog stick's X value
    double rightJoyX = j_stick_driver.getRawAxis(4);   // Grab right analog stick's Y value
    leftJoyY = LB_driver_pressed ? leftJoyY / 2 : leftJoyY;
    rightJoyX = LB_driver_pressed ? rightJoyX / 2 : rightJoyX;
    m_robotDrive.arcadeDrive(leftJoyY, rightJoyX);       // Drive the robot using arcade drive

    
    // Testing Cargo Lift Code //
    if (X_pressed) {
      m_lift_cargo_left.set(-0.65);
      m_lift_cargo_right.set(0.65);
    }
    else if (Y_pressed) {
      m_lift_cargo_left.set(0.65);
      m_lift_cargo_right.set(-0.65);
    }
    else {
      m_lift_cargo_left.stopMotor();
      m_lift_cargo_right.stopMotor();
    }
    // Testing Cargo Shooter //
    if (A_pressed) {
      m_cargo.set(1);
    } 
    else if (B_pressed) {
      m_cargo.set(-1);
    } 
    else {
      m_cargo.stopMotor();
    } 

    // Testing Hatch Pneumatics Code // 
    if (Back_pressed) {
      // Forward Pneumatics //
      doubleSolenoid_Hatch.set(Value.kForward); // Right solenoid go forward
    } 
    else if (Start_pressed) {
      doubleSolenoid_Hatch.set(Value.kReverse); // Right solenoid go backward
    } 
    else {
      doubleSolenoid_Hatch.set(Value.kOff); // Right solenoid turns off
    } 

    // Testing Hatch Code //
    if (LB_pressed) {
      m_hatch.set(-.6);   // Hatch motor goes full power forward when A is pressed
    } 
    else if (RB_pressed) {
      m_hatch.set(.6);  // Hatch motor goes full power forward when B is pressed
    } 
    else {
      m_hatch.stopMotor();  // Hatch motor stops when no button is pressed.
    }

    // Testing Front Landing Gears //
     if (X_driver_pressed) {
      // Forward Pneumatics //
      doubleSolenoid_Front.set(Value.kForward);  // Left solenoid go forward
    } 
    else if (Y_driver_pressed) {
      doubleSolenoid_Front.set(Value.kReverse);  // Left solenoid go forward

    } 
    else {
      doubleSolenoid_Front.set(Value.kOff);  // Left solenoid turns off
    }

    // Testing Back Landing Gears //
    if (A_driver_pressed) {
      doubleSolenoid_Back.set(Value.kForward); // Right solenoid go forward
    }
    else if (B_driver_pressed) {
      doubleSolenoid_Back.set(Value.kReverse); // Right solenoid go backward
    }
    else {
      doubleSolenoid_Back.set(Value.kOff); // Right solenoid turns off
    }  

  }
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    // This is used for testing //

    // Testing Joystick Motor Control //
    // double x = j_stick_driver.getX();
    // double y = j_stick_driver.getY();
    // m_robotDrive.arcadeDrive(j_stick_driver.getY(), j_stick_driver.getX());

    // Testing Pneumatics and Double Solenoid Code //
    // boolean X_pressed = j_stick_control_X.get();  // Get X pressed value
    // boolean Y_pressed = j_stick_control_Y.get();  // Get Y pressed value
    // boolean A_pressed = j_stick_control_A.get();  // Get A pressed value
    // boolean B_pressed = j_stick_control_B.get();  // Get B pressed value

    // if(X_pressed){
    //   // Forward Pneumatics //
    //   doubleSolenoid_Front.set(Value.kForward);
    // }
    // else if(Y_pressed){
    //   // Reverse Pneumatics //
    //   doubleSolenoid_Front.set(Value.kReverse);
    // }
    // else if(A_pressed){
    //   // Off Pneumatics //
    //   doubleSolenoid_Front.set(Value.kOff);
    // }
    // else{
    //   // Toggle Compressor //
    //   boolean compressor_enabled = !compressor.enabled();
    //   compressor.setClosedLoopControl(compressor_enabled);
    // } 

  }
}
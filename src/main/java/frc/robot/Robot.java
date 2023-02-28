// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
// import frc.robot.Core.Scheduler;
import frc.robot.Drive.*;
import frc.robot.Util.*;
import frc.robot.Motor.SparkMax;

import org.javatuples.Pair;
//+++This is Alex from Team 980.  Unless asked I won't actuall make any changes to your code.  Instead I will put comments suggesting code changes or recommendations on how to do something
//+++You will know it was me by +++ leading the line


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Drive drive;
  PS4Controller con;
  DriveSidePD left;
  DriveSidePD right;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    { //+++This set of braces are unnecessary
      // Drive initalization 
      var leftFront = new SparkMax(3, true);
      var leftBack = new SparkMax(1, true);
      var leftTop = new SparkMax(2, false);
      var rightFront = new SparkMax(7, false);
      var rightBack = new SparkMax(6, false);
      var rightTop = new SparkMax(4, true);

      DriveSide left = new DriveSide(leftFront, leftBack, leftTop);
      DriveSide right = new DriveSide(rightFront, rightBack, rightTop);
      /*
       * +++You are reinventing the wheel here.  Most of the classes you are writing already exist in the WPILib.
       * MotorControllerGroup class takes multiple motors and turns them into a single item to use:
       * MotorControllerGroup leftDrive = new MotorControllerGroup(leftTop, leftBack, leftFront);
       * this will allow you to invert an entire side rather than individual motors.  So only invert the top motors and then use:
       * rightDrive.setInverted(true);
       */
      

      final var HIGHGEARCONTROLLER = new PDController(300, 0);
      final var LOWGEARCONTROLLER = new PDController(300, 0);
      /*
       * +++There is also a PIDController class which handles all the PID stuff more efficiently than what you have
       * PIDController lowGearController = new PIDController(300, 0, 0);
       */

      
      this.left = new DriveSidePD(left, LOWGEARCONTROLLER, HIGHGEARCONTROLLER);
      this.right = new DriveSidePD(left, LOWGEARCONTROLLER, HIGHGEARCONTROLLER);

      this.drive = new Drive(left, right);
      /*
       * +++ There is a Tank Drive setup premade as well this is the setup for it, called DifferentialDrive:
       * DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);
       * see TeleopPeriodic below for how you use this to drive the robot
       */
      
    }
    this.con = new PS4Controller(0);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Scheduler.getInstance().clear();
    drive.shiftLowGear();
    left.reset();
    right.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Scheduler.getInstance().tick();
    // left.incrementTarget(0.003);
    left.tick(null);
  }

  @Override
  public void teleopInit() {
    // Scheduler.getInstance().clear();
    drive.shiftLowGear();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Scheduler.getInstance().tick();

    final double deadzone = 0.03;
    final double turnCurveIntensity = 7;
    final double pwrCurveIntensity = 5;
    final Pair<Double, Double> powers = ScaleInput.normalize(ScaleInput.scale(
        con.getLeftY(),
        con.getRightY(),
        deadzone,
        turnCurveIntensity,
        pwrCurveIntensity));
    drive.setPower(powers.getValue0(), powers.getValue1());
    /*
     * +++ Here is how to use the Differential Drive:
     * drive.tankDrive(leftSpeed, rightSpeed);
     * leftSpeed and rightSpeed take values between -1 and 1, this is what the sticks on your controller put out so it makes for a really clean and simple drive system.
     * The right side will have to be inverted for tankDrive, which I showed above
     * if you want to use the PID to control this, you can by making sure the output is between -1 and 1.  However, if you want to use PID to control your drive velocity
     * you will need a feed forward in addition to the PIDController output
     * SimpleMotorFeedforward ffLowLeft = new SimpleMotorFeedforward(ksLow, 1.0/maxVelocityLow); placed in the drive initialization area of the contructor above
     * ksLow is a constant determining how much power it takes to overcome static friction on the wheels, I have been using .05 for that
     * the second constant is kV and it is detemined by the max velocity of the robot (so you need a feed forward for each gear) and the top of the fraction is the absolute 
     * value of maximum input value, which in the case of tank drive is 1 but could be any output you want to use.  
     * You will need a feed forward for each side in each gear. To use it, this is the call:
     * ffLowLeft.calculate(setpoint);
     * setpoint is the value from the controller joystick from -1 to 1.  You would use it in the tank drive call:
     * drive.tankDrive(ffLowLeft.calculate(setpoint), ffLowRight.calculate(setpoint));
     * and if you have an output from a PID controller you would put that in too:
     * drive.tankDrive(PIDLeftOutput + ffLowLeft.calculate(setpoint), PIDRightOutput + ffLowRight.calculate(setpoint));
     * 
     * If you want me to help you convert over to the WPILib version of the PIDController, let me know.  Since we use true PID control and arcade drive, the Team 980
     * code is structured way differently so I would have to show you how to do this in your Timed Robot structure
     */
  
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Scheduler.getInstance().clear();
    drive.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  // ! we won't run any code beyond this point

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}

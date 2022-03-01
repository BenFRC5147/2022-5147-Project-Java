package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Robot extends TimedRobot {
  
  //Defines the drivetrain
  WPI_TalonSRX frontRight = new WPI_TalonSRX(2);
  WPI_TalonSRX frontLeft = new WPI_TalonSRX(0);
  WPI_TalonSRX backRight = new WPI_TalonSRX(3);
  WPI_TalonSRX backLeft = new WPI_TalonSRX(1);
  DifferentialDrive mainDrive = new DifferentialDrive(frontLeft, frontRight);

  //Defines the manipulator
  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  WPI_TalonSRX intake = new WPI_TalonSRX(4);

  //Defines the joystick
  Joystick driverController = new Joystick(0);

  //Constants for controlling the arm. consider tuning these for your particular robot
  final double armHoldUp = 0.08;
  final double armHoldDown = 0.13;
  final double armTravel = 0.125;
  final double armTimeUp = 5;
  final double armTimeDown = 8;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0;
  double autoStart = 0;
  boolean goForAuto = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Sets back motors to be slaved to the front motors
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    backLeft.setInverted(false);
    backRight.setInverted(true);
    frontRight.setInverted(true);
    frontLeft.setInverted(true);
    arm.setInverted(false);
    arm.setIdleMode(IdleMode.kBrake);
    arm.burnFlash();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //arm control code. same as in teleop
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldUp);
      }
    }
    
    //get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if(goForAuto){
      //series of timed events making up the flow of auto
      if(autoTimeElapsed < 3){
        //spit out the ball for three seconds
        intake.set(ControlMode.PercentOutput, -1);
      }else if(autoTimeElapsed < 6){
        //stop spitting out the ball and drive backwards *slowly* for three seconds
        intake.set(ControlMode.PercentOutput, 0);
        mainDrive.arcadeDrive(-0.3, 0);
      }else{
        //do nothing for the rest of auto
        intake.set(ControlMode.PercentOutput, 0);
        mainDrive.arcadeDrive(0, 0);
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Set up arcade steer

    mainDrive.arcadeDrive(-driverController.getRawAxis(1), driverController.getRawAxis(0));

    //Intake controls
    if(driverController.getRawButton(12)){
      intake.set(ControlMode.PercentOutput, 1);;
    }
    else if(driverController.getRawButton(7)){
      intake.set(ControlMode.PercentOutput, -1);
    }
    else{
      intake.set(ControlMode.PercentOutput, 0);
    }

    //Arm Controls
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldDown);
      }
    }
  
    if(driverController.getRawButtonPressed(6) && !armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
    }
    else if(driverController.getRawButtonPressed(8) && armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
    }  

  }

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    mainDrive.arcadeDrive(0, 0);
    arm.set(0);
    intake.set(ControlMode.PercentOutput, 0);
  }
    
}

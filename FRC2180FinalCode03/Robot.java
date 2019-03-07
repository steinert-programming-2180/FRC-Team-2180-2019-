package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.cscore.UsbCamera;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  int counter = 0;
  boolean iterate;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  DoubleSolenoid hatchMechanism = new DoubleSolenoid(2, 3); //Sets up pneumatics
  Compressor compressor = new Compressor();

  int armTarget = 0;
  int wristTarget = 0;

/*
  public static DigitalInput armLimitBottom = new DigitalInput(0);
  public static DigitalInput armLimitTop = new DigitalInput(1);
*/
  public static double armOutput = 0.0;
  //public static boolean armCanGoUp = false;
  //public static boolean armCanGoDown = false;

  public static boolean sensorInPhase, motorInverted; 

  double l;
  double r;

  double gyroHeading;
  double desiredHeading;

  double angleDifference;
  double turn;

  TalonSRX wristLeft = new TalonSRX(13);
  TalonSRX wristRight = new TalonSRX(14);
  TalonSRX intake = new TalonSRX(15);
  
  TalonSRX armLeft = new TalonSRX(11);
  TalonSRX armRight = new TalonSRX(12);

  TalonSRX left1 = new TalonSRX(01);
  TalonSRX left2 = new TalonSRX(02);
  TalonSRX left3 = new TalonSRX(03);

  TalonSRX[] left = {left1, left2, left3};

  TalonSRX right1 = new TalonSRX(04);
  TalonSRX right2 = new TalonSRX(05);
  TalonSRX right3 = new TalonSRX(06);

  TalonSRX LeftArm = new TalonSRX(11);
  TalonSRX RightArm = new TalonSRX(12);

  TalonSRX LeftWrist = new TalonSRX(13);
  TalonSRX RightWrist = new TalonSRX(14);
  
  TalonSRX Intake = new TalonSRX(15);

  TalonSRX[] right = {right1, right2, right3};

  Joystick stick = new Joystick(0);
  Joystick stick2 = new Joystick(1);
  Joystick stick3 = new Joystick(2);
  Joystick stick4 = new Joystick(3);

  boolean foward = true;
  boolean previousButton = false;

  boolean pistonsOut = false;

  int multiplier = 1; //Reverses drive train we think

  UsbCamera cameraFront = new UsbCamera("camFront", 0);
  UsbCamera cameraBack = new UsbCamera ("camBack", 1);
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());

    armLeft.configFactoryDefault();
    armRight.configFactoryDefault();

    wristLeft.configFactoryDefault();
    wristRight.configFactoryDefault();

    // wristLeft.setInverted(false);
    // wristRight.setInverted(true);

    // wristRight.follow(wristLeft);

    setupWristPID(0);
    setupArmPID(0);

    armLeft.setSelectedSensorPosition(0);
    wristLeft.setSelectedSensorPosition(0);

    SmartDashboard.putData("Auto mode", m_chooser);
    armLeft.setSelectedSensorPosition(0);
    wristLeft.setSelectedSensorPosition(0);
  }

  public void setMotors (TalonSRX[] left, TalonSRX[] right, double Lspeed, double Rspeed, boolean foward) {
    if (foward) {
      for (TalonSRX x : left) {
        x.set(ControlMode.PercentOutput, Lspeed);
      } for (TalonSRX x : right) {
        x.set(ControlMode.PercentOutput, -1 * Rspeed);
      }
    } else {
      for (TalonSRX x : right) {
        x.set(ControlMode.PercentOutput, Lspeed);
      } for (TalonSRX x : left) {
        x.set(ControlMode.PercentOutput, -1 * Rspeed);
      }
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    periodicInit();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */
    
     if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    periodicMotion();
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    periodicInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    periodicMotion(); //Calls function at the bottom of the file, for organizational purposes
    Scheduler.getInstance().run();
  }

    /**
     * This function is called periodically during test mode.
     */
    @Override
  public void testPeriodic() {}

  public void setupArmPID(int pidSlot) {
    sensorInPhase = false;
    motorInverted = true;

    armRight.follow(armLeft);

    
    armLeft.configFactoryDefault();
    armLeft.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    armLeft.configFeedbackNotContinuous(true, 10);

    armLeft.setInverted(motorInverted);
    armRight.setInverted(motorInverted);

    armLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    armLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
      
    armLeft.configNominalOutputForward(0, 10);
    armLeft.configNominalOutputReverse(0, 10);
    armLeft.configPeakOutputForward(Constants.maxArmSpeed, 10);
    armLeft.configPeakOutputReverse(-Constants.maxArmSpeed, 10);
      
    armLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    armLeft.config_kF(0, 0.0, 10);
    armLeft.config_kP(0, Constants.armKP, 10);
    armLeft.config_kI(0, Constants.armKI, 10);
    armLeft.config_kD(0, Constants.armKD, 10);
      
    armLeft.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    armLeft.configMotionAcceleration(6000, Constants.kTimeoutMs);
      
    armLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void setupWristPID(int pidSlot) {
    sensorInPhase = true;
    motorInverted = false;

    wristLeft.setInverted(motorInverted);
    wristRight.setInverted(motorInverted);

    wristLeft.configFactoryDefault();
    wristLeft.configSelectedFeedbackSensor(FeedbackDevice.Analog
                                            ,Constants.kPIDLoopIdx 
                                            ,Constants.kTimeoutMs);
    wristLeft.configFeedbackNotContinuous(true, 10);

    //wristLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    //wristLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
      
    wristLeft.configNominalOutputForward(0, 10);
    wristLeft.configNominalOutputReverse(0, 10);
    wristLeft.configPeakOutputForward(Constants.maxWristSpeed, 10);
    wristLeft.configPeakOutputReverse(-Constants.maxWristSpeed, 10);
      
    wristLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    wristLeft.config_kF(0, 0.0, 10);
    wristLeft.config_kP(0, Constants.wristKP, 10);
    wristLeft.config_kI(0, Constants.wristKI, 10);
    wristLeft.config_kD(0, Constants.wristKD, 10);
      
    wristLeft.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    wristLeft.configMotionAcceleration(6000, Constants.kTimeoutMs);
      
    wristLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void periodicInit () {
    counter = 0;

    armRight.setNeutralMode(NeutralMode.Brake);
    armLeft.setNeutralMode(NeutralMode.Brake);
    wristRight.setNeutralMode(NeutralMode.Brake);
    wristLeft.setNeutralMode(NeutralMode.Brake);
    intake.setNeutralMode(NeutralMode.Brake);

    armRight.follow(armLeft);   
    
    armLeft.setInverted(false);

    wristLeft.setInverted(false);
    wristRight.setInverted(false);
    
    compressor.start();
    hatchMechanism.set(DoubleSolenoid.Value.kOff);
  }

  public void periodicMotion () {
    if(stick.getRawButtonPressed(1)) {
      foward = !foward;
    } //SmartDashboard.putNumber("Multiplier", multiplier);

    if (foward) {
      CameraServer.getInstance().startAutomaticCapture(0);
    } else {
      CameraServer.getInstance().startAutomaticCapture(1);
    }

    double rightSpeed = -stick.getRawAxis(1) * multiplier;
    double leftSpeed = -stick2.getRawAxis(1) * multiplier;
    
    SmartDashboard.putBoolean("Forward", foward);
    
    setMotors(left, right, leftSpeed, rightSpeed, foward);

    boolean wristRightinverted = true;
    
    //------------------------------------------------------------------------
    //TODO: Comment all of these with their actual positions
    if (stick3.getRawButton(3)) {
      armTarget = 70; //70
      wristTarget = 95; //95
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    } 
    
    if (stick3.getRawButton(5)) {
      armTarget = 0;  //0
      wristTarget = 63;  //63
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    } 

    if (stick3.getRawButton(8)){
      armTarget = 220;  //220
      wristTarget = -50;  //-50
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
      //hatchMechanism.set(DoubleSolenoid.Value.kForward);  Depreciated PizzaStubs, remove or change as needed
      pistonsOut = true;
    }
    if (stick3.getRawButton(10)){
      armTarget = 10; //10
      wristTarget = 10;  //10
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick3.getRawButton(2)) {
      armTarget = 73;  //73
      wristTarget = 53;  //53
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }
    
    if (stick3.getRawButton(11)) {
      armTarget = 225;  //225
      wristTarget = 50;  //50
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }
    
    if (stick.getRawButton(2)) {
      armTarget = 37;  //37
      wristTarget = 18;  //18
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick.getRawButton(3)) {
      armTarget = 65;  //65
      wristTarget = 18;  //18
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick3.getRawButton(9)) {
      armTarget = -50;  //-50
      wristTarget = -5;  //-5
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick3.getRawButtonPressed(1)) { //Pneumatics
      pistonsOut = !pistonsOut;
      if(pistonsOut)
        hatchMechanism.set(DoubleSolenoid.Value.kForward);
      else
        hatchMechanism.set(DoubleSolenoid.Value.kReverse);
    }

    SmartDashboard.putNumber("Wrist motor output", wristLeft.getMotorOutputPercent());

    if (stick3.getRawButton(6)) {  //Intake
      Intake.set(ControlMode.PercentOutput, 1.0);
    } else if (stick3.getRawButton(7)) {
      Intake.set(ControlMode.PercentOutput, -1.0);
    } else {
      Intake.set(ControlMode.PercentOutput, 0);
    }

    //----------------------------------------------
    //Manual Control

    counter++;
    iterate = (counter % 8) == 0;

    if (iterate) {
      if (stick4.getRawButton(6)) {  //Far-Left button on base
        wristTarget++;
      } else if (stick4.getRawButton(7)) { //Close-Left button on base
        wristTarget--;
      }
  
      //Manual arm control
      if (stick4.getRawButton(11)) { //Far-Right button on base
        armTarget++;
      } else if (stick4.getRawButton(7)) { //Close-Right button on base
        armTarget--;
      }
    }

    armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
    wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);

    SmartDashboard.putNumber("ArmPot Position", armLeft.getSelectedSensorPosition()); //Added by Nikhil, reports Pot position
    SmartDashboard.putNumber("WristPot Position", wristLeft.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Pistons Out", pistonsOut);
  }
}

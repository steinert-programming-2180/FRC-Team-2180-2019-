/*
To Whomever May Be Testing This Code,
I have commented everything that needs to be tested, and what should be doing what.
If you need help before 8:30 pm, just message me and I'll explain what I can.  After that I'll be sleeping.
Please test:
  -Pneumatics
    -Hatch Mechanism
      -Lines 88 and 89 initialize
      -Lines 361 and 362 perform init tasks
      -Lines 452-458 actually manipulate the pistons
  -Camera Feeds
    -Lines 152-160 initialize things
    -Lines 172-180 cast to the driver station
    -Lines 379-383 switches feed when trigger is pushed
  -Positions of pre-sets
    -Lines 387-457 contain all presets.  Just go through them one by one and comment the position they encode.
     The zeroing issue has been corrected, but all positions are listed in the conditionals with their original 
     values, and modified by an offset constant.  Thus, 0 is roughly where the arm starts, although it may not be
     exact.
Note that line numbers may not be exact, as modifications may alter them.  All should be in the ballpark, however.
The code is reorganized, so below is a table of contents.

Packages: 38
Initializations: 79
Autonomous: 224
TeleOp: 251
setUpArmPID (This method organizes all of the setup for the arm's PID): 278
setUpWristPID (Same as above, but for the wrist): 312
periodicInit (This method organizes init for both Sandstorm and TeleOp): 345
periodicMotion (This method organizes the periodic section for Sandstorm and TeleOp): 365
setMotors (This method organizes all the motors and synchs them): 498

The endgame is not programmed yet, as I didn't get a chance to actually see how it was wired.  If you have time,
you should add it.

-Ravi "Lord Large Lard Lad" D'Elia
*/
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
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera;
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

  // Sets up pneumatics
  DoubleSolenoid hatchMechanism = new DoubleSolenoid(2, 3); 
  DoubleSolenoid hatchOut = new DoubleSolenoid(4, 5);
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

  boolean grabPistonsOut = false;
  boolean hatchOutOut = false;

  int multiplier = 1; //Reverses drive train we think
  /*
  VideoCamera cameraFront = new UsbCamera("Camera0", 0);
  VideoCamera cameraBack = new UsbCamera("Camera1", 1);
  MjpegServer server = new MjpegServer("Output to dashboard", 1181);

  CameraServer sendServer = CameraServer.getInstance();

  VideoCamera[] cameras = {cameraFront, cameraBack};

  int cameraCounter = 0;
  */
  UsbCamera camBack = new UsbCamera("CamBack", 1);
  //UsbCamera camFront= new UsbCamera("CameFront", 0);

  CameraServer serverBack = CameraServer.getInstance();
  //CameraServer serverFront = CameraServer.getInstance();
  


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    /*
    server.setSource(cameraFront); //This is what defaults the camera
    sendServer.addServer(server); //This adds the MJpegServer to CameraServer so it can be sent to the dashboard
    sendServer.startAutomaticCapture(); //This actually sends it
    */
    armLeft.configFactoryDefault();//These should fix the zeroing issue
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

    serverBack.startAutomaticCapture(0);
    //serverFront.startAutomaticCapture(1);
    //camFront.setFPS(30);
    camBack.setFPS(30);
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
      
    // wristLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void periodicInit () {
    armRight.setNeutralMode(NeutralMode.Brake);
    armLeft.setNeutralMode(NeutralMode.Brake);
    wristRight.setNeutralMode(NeutralMode.Brake);
    wristLeft.setNeutralMode(NeutralMode.Brake);
    intake.setNeutralMode(NeutralMode.Brake);

    armRight.follow(armLeft);   
    
    armLeft.setInverted(false);

    wristLeft.setInverted(false);
    wristRight.setInverted(false);
    
    compressor.start();  //Sets up the pneumatics and prepares for use
    hatchMechanism.set(DoubleSolenoid.Value.kOff);
    hatchOut.set(DoubleSolenoid.Value.kOff);

    // armLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void periodicMotion () {
    if(stick.getRawButtonPressed(1) && !previousButton) {
      foward = !foward;
    } //SmartDashboard.putNumber("Multiplier", multiplier);

    double rightSpeed = -stick.getRawAxis(1) * multiplier;
    double leftSpeed = -stick2.getRawAxis(1) * multiplier;
    
    SmartDashboard.putBoolean("Forward", foward);
    
    setMotors(left, right, leftSpeed, rightSpeed, foward);

    boolean wristRightinverted = true;
    
    //------------------------------------------------------------------------
    //TODO: Comment all of these with their actual positions
    
    // pick up ball from floor
    if (stick3.getRawButton(3)) {
      armTarget = 56; //70
      wristTarget = 95; //95
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    } 
    
    if (stick3.getRawButton(5)) {
      armTarget = 0;  //0
      wristTarget = 63;  //63
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    } 

    // spit ball backwards
    if (stick3.getRawButton(8)){
      armTarget = 200;  //220
      wristTarget = -40;  //-50
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
      //hatchMechanism.set(DoubleSolenoid.Value.kForward);  Depreciated PizzaStubs, remove or change as needed
      grabPistonsOut = true;
    }
    if (stick3.getRawButton(10)){
      armTarget = 10; //10
      wristTarget = 10;  //10
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick3.getRawButton(2)) {
      armTarget = 73;  //73
      wristTarget = 53;  //53
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }
    
    if (stick3.getRawButton(11)) {
      armTarget = 225;  //225
      wristTarget = 50;  //50
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }
    
    if (stick.getRawButton(2)) {
      armTarget = 37;  //37
      wristTarget = 18;  //18
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick.getRawButton(3)) {
      armTarget = 65;  //65
      wristTarget = 18;  //18
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick3.getRawButton(9)) {
      armTarget = -50;  //-50
      wristTarget = -5;  //-5
      armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
      wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);
      wristRight.setInverted(wristRightinverted);
      wristRight.follow(wristLeft);
    }

    if (stick4.getRawButtonPressed(1)) { //Pneumatics: UpGrab
      grabPistonsOut = !grabPistonsOut;
      if(grabPistonsOut) {
        hatchMechanism.set(DoubleSolenoid.Value.kForward);
      } else {
        hatchMechanism.set(DoubleSolenoid.Value.kReverse);
      }
    } if (stick4.getRawButtonPressed(2)) { //Pneumatics: HatchOut
      hatchOutOut = !hatchOutOut;
      if(hatchOutOut) {
        hatchOut.set(DoubleSolenoid.Value.kForward);
      }
      else{
        hatchOut.set(DoubleSolenoid.Value.kReverse);
      }
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

    // counter++;
    // iterate = (counter % 8) == 0;

    // if (iterate) {
    //   if (stick4.getRawButton(6)) {  //Far-Left button on base
    //     wristTarget++;
    //   } else if (stick4.getRawButton(7)) { //Close-Left button on base
    //     wristTarget--;
    //   }
  
    //   //Manual arm control
    //   if (stick4.getRawButton(11)) { //Far-Right button on base
    //     armTarget++;
    //   } else if (stick4.getRawButton(7)) { //Close-Right button on base
    //     armTarget--;
    //   }
    // }

    // armLeft.set(ControlMode.MotionMagic, armTarget + Constants.armOffset);
    // wristLeft.set(ControlMode.MotionMagic, wristTarget + Constants.wristOffset);

    SmartDashboard.putNumber("ArmPot Position", armLeft.getSelectedSensorPosition()); //Added by Nikhil, reports Pot position
    SmartDashboard.putNumber("WristPot Position", wristLeft.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Pistons Out", grabPistonsOut);
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
}

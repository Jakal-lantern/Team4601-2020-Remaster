/*
*2020 Code
*/ 


// Limelight API info -- http://docs.limelightvision.io/en/latest/networktables_api.html

// Shou
/* changelog: 
    Feb 30, 2020 Dr. C. 
          - added test code and hardware (currently on the MXP port of the Roborio)
          for testing a (first of two) simple hardware auton switch. 
          - Also added gyro instance 

    March 5, 2020 Alex
          - added the library for the ADIS16470. Should use that instead
          of the default gyro inside the RoboRIO. Documentation for it
          can be found at https://juchong.github.io/ADIS16470-RoboRIO-Driver/classfrc_1_1_a_d_i_s16470___i_m_u.html
          - added ADIS instance
          - added calibrate function in autoninit
          - changed values in autonperiodic so that robot drives straight
          using gyro in ADIS
          - converted spark controllers to victor spx

      March 6, 2020 Alex
          - added functional code to many operations
          - started work on fixing bugs in auton

*/ 
#include <iostream>
#include <memory>
#include <frc/WPILib.h>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <unistd.h>
#include <frc/IterativeRobot.h>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/PWM.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/util/Color.h>
#include <string>
#include <math.h>
#include <adi/ADIS16470_IMU.h>
#include <frc/Timer.h>

//#include "rev/ColorSensorV3.h"
//#include "rev/ColorMatch.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

using namespace frc;

/*
* -Continue to test limelight distance code
* -Make function to find angle using distance
* -Complete auton / test for fine tuning
*/ 

class Robot : public TimedRobot {

  /*
  * Initialization
  */

  // Auton
  LiveWindow *lw = LiveWindow::GetInstance();
  //SendableChooser chooser = new SendableChooser();
  int autonSwitch;
  bool autonIsActive;
  frc::Timer *timer =new Timer();

  // Motor Controller
  PWMVictorSPX *light =new PWMVictorSPX(0);
  PWMVictorSPX *fRight =new PWMVictorSPX(1);
  PWMVictorSPX *fLeft =new PWMVictorSPX(2);
  PWMVictorSPX *turret =new PWMVictorSPX(3);
  PWMVictorSPX *arm =new PWMVictorSPX(4);
  PWMVictorSPX *turretShoot =new PWMVictorSPX(5);
  
  // Joystick
  DifferentialDrive m_robotDrive{*fRight, *fLeft};
  Joystick m_stick{0};
  Joystick m_stick2{1};
  XboxController controller1{2};

  // Camera
  cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture(0);
  // Variables
  std::string colorString;
  bool canElevate = true,driveState = true;
  float steeringAdjust, controlConstant = -0.1f;
  double throttle1, pi = 4.0*atan(1.0), g=9.8, driveCommand;
  double kP = 0.005;
  double kAngleSetpoint = 0.0;
 
  // digital input
  frc::DigitalInput lSwitch1{12};
  frc::DigitalInput lSwitch2{14};

  // digital output
  frc::DigitalOutput LED1{10};

  // Sensors
  //frc::BuiltInAccelerometer accel;
  // frc::AnalogGyro gyro1{3};
  frc::ADIS16470_IMU imu{frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0, frc::ADIS16470CalibrationTime::_4s};

  // Limelight Variables
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  bool LimelightHasTarget;
  double LimelightTurnCmd;
  double LimelightDriveCmd;
  double clamp(double in, double minval, double maxval) {
    if (in > maxval) return maxval;
    if (in < minval) return maxval;
    return in;
  }
  
  /*
  * Main Robot Functions
  */
  void RobotInit()
  {
    // gyro1.Reset();
    // gyro1.InitGyro();
  } 
  // Ran before auton but cannot initialize values inside it
  void AutonomousInit() {
    //  if(autonomousCommand != NULL) autonomousCommand->Start();
    // read auton switch to determine behavior 
    imu.Calibrate();
    timer->Start();
    autonIsActive = true;
  }

  // Auton Function
  void AutonomousPeriodic() {  
    //  Scheduler::GetInstance()->Run();

    //see where we are based on where tape is relative to us
    /* 
    *  Switch behaviour based on starting position
    *  0 = right
    *  1 = middle
    *  2 = left
    */
    switch(lSwitch1.Get()) {
      case (0):
        autonSwitch = 0;
        break;

      case (1):
        if (lSwitch2.Get() == 0) {
          autonSwitch = 1;
        }
        else {
          autonSwitch = 2;
        }
        break;
    }

    // Behavior I : Drive back, pickup balls, lineup, shoot
    double turningValue = (kAngleSetpoint - imu.GetAngle()) * kP;
    double timerValue = timer->Get();
    double Kp, Kint, error, errorInt, target, r=0.05;
    if (autonSwitch == 0) {
      // Pickup Balls
      if (timerValue < 2.0) {
        driveCommand = -0.75;
        m_robotDrive.ArcadeDrive(driveCommand, turningValue);
      }

      if (timerValue > 2.5 && timerValue < 4) {
        driveCommand = 0.75;
        m_robotDrive.ArcadeDrive(driveCommand, turningValue);
      }
      

      std::string AS1, AS2;   
      AS1 = "1"; 
      AS2 = "1"; 
      if(lSwitch1.Get()==0) {
      AS1 = "0"; 
      } 
      if(lSwitch2.Get()==0) {
      AS2= "0"; 
      } 

      Kp = 0.05/90.0; 
      Kint = 0.250; 
      errorInt=0.0; 
      if (timerValue > 4.5 && timerValue < 5) {
          target = 90.0; 
          error = abs(imu.GetAngle())-target; 
          errorInt = (1.0-r)*errorInt+r*error; 
          driveCommand = Kp*error+Kint*errorInt;

        if(abs(driveCommand)<0.5) {
          driveCommand=0.5*driveCommand/abs(driveCommand);
        }
        else if(abs(driveCommand)>1) {
          driveCommand=1*driveCommand/abs(driveCommand);
        }
          m_robotDrive.TankDrive(driveCommand, -driveCommand);
      }
      else {
        m_robotDrive.TankDrive(0.0, 0.0);
      }

      // Turn turret

      // Drive until limelight center
      if (timerValue > 5 && timerValue < 8.5) {
        AutonLimelightTracking();
        m_robotDrive.TankDrive(LimelightTurnCmd, -LimelightTurnCmd);
      }


    }
    // Behavior II : drive forward, shoot
    else if (autonSwitch == 1) {
      // Drive forward
      if (timerValue < 2.0) {
        driveCommand = 0.75;
        m_robotDrive.ArcadeDrive(driveCommand, turningValue);
      }
      // Shoot
      else {

      }

      
    }
    // Behavior III : drive forward, lineup, shoot
    else if (autonSwitch == 2) {
      // Drive forward
      if (timerValue < 1.0) {
        driveCommand = 0.75;
        m_robotDrive.ArcadeDrive(driveCommand, turningValue);
      }

      // Aim
      if (timerValue > 1.5 && timerValue < 2.5) {
        UpdateLimelightTracking();
      }

      // Shoot
      if (timerValue > 3.5) {

      }
    }
    
}





  // Teleop Function
  void TeleopPeriodic() { 
    autonIsActive = false;
    /*
    * Driver Controls
    */

    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());

    //Drive with tank style
    //m_robotDrive.TankDrive(m_stick2.GetY(), m_stick.GetY());

    if (m_stick2.GetRawButton(13)) {
      driveState = true;
    }

    if (m_stick2.GetRawButton(14)) {
      driveState = false;
    }


    if (driveState) {
      UpdateLimelightTracking();

      if (controller1.GetAButton()) {
        if (LimelightHasTarget) {
          m_robotDrive.ArcadeDrive(LimelightDriveCmd, LimelightTurnCmd);
        }
        else {
          m_robotDrive.ArcadeDrive(0.0, 0.0);
        }
      }
      else {
        double fwd = m_stick.GetY();
        double turn = m_stick.GetX();
        // fwd *= 0.7f;
        // turn *= 0.7f;
        throttle1 = m_stick.GetThrottle();

        if (throttle1 > .5) {
          m_robotDrive.ArcadeDrive(fwd, -turn);
        }
        else if (throttle1 < -.5) {
          m_robotDrive.ArcadeDrive(-fwd, turn);
        }
        else {
          m_robotDrive.ArcadeDrive(-fwd / 2, turn / 1.7);
        }
      }
    }
    else {
      // Tank drive auton code
      // This could be working. Needs more testing
      float currentDistance = EstimateDistance();
      float desiredDistance = 500;
      int distanceThresh = 5;

      if (controller1.GetBButton()) {
        float drivingDistance = desiredDistance - currentDistance;

        if (drivingDistance < distanceThresh || drivingDistance > -distanceThresh) {
          driveCommand = 0;
        }if (drivingDistance < desiredDistance) {
          driveCommand = 0.5;
        } 
        else if (drivingDistance > desiredDistance) {
          driveCommand = -0.5;
        }

        m_robotDrive.TankDrive(driveCommand, driveCommand);
      }
    }

    // Arm Piston
    if (m_stick.GetRawButton(3)) {
      // Input piston code here
      SmartDashboard::PutString("DB/String 1", "Arm Position - 1");
    }
    else if (m_stick.GetRawButton(4)) {
      // Input piston code here
      SmartDashboard::PutString("DB/String 1", "Arm Position - 2");
    }

    // Limelight Lock On
    if (m_stick.GetRawButton(2)) {
      UpdateLimelightTracking();
    }

    // Limelight Zoom
    if (m_stick.GetRawButton(8)) {
      // Input default zoom
      table->PutNumber("pipeline", 0);
      SmartDashboard::PutString("DB/String 5", "Cam: 1");
    }
    else if (m_stick.GetRawButton(9)) {
      // Input level 2 zoom
      table->PutNumber("pipeline", 1);
      SmartDashboard::PutString("DB/String 5", "Cam: 2");
    }
    else if (m_stick.GetRawButton(10)) {
      // Input level 3 zoom
      table->PutNumber("pipeline", 2);
      SmartDashboard::PutString("DB/String 5", "Cam: 3");
    }
    else if (m_stick.GetRawButton(7)) {
      // Driver cam 1
      table->PutNumber("pipeline", 3);
      SmartDashboard::PutString("DB/String 5", "Cam: 4");
    }
    else if (m_stick.GetRawButton(6)) {
      // Driver cam 2
      table->PutNumber("pipeline", 4);
      SmartDashboard::PutString("DB/String 5", "Cam: 5");
    }

    /*
    * Co-Pilot Controls
    */

    // Elevator Code
    if (canElevate) {
      switch (controller1.GetPOV()) {
        case 0:
          // Up Button
          SmartDashboard::PutString("DB/String 3", "DPad: Up");
          break;
        
        case 90:
          // Right Button
          SmartDashboard::PutString("DB/String 3", "DPad: Right");
          break;

        case 180:
          // Down Button
          SmartDashboard::PutString("DB/String 3", "DPad: Down");
          break;

        case 270:
          // Left Button
          SmartDashboard::PutString("DB/String 3", "DPad: Left");
          break;

        default:
          SmartDashboard::PutString("DB/String 3", "DPad:");
          break;
      }
    }

    // Shooting Code
    // if (controller1.GetRawAxis(2) && controller1.GetRawAxis(3)) {
    //   // Firing Code
    //   SmartDashboard::PutString("DB/String 6", "FIRING MA LAZUR");
    // }
    // else if (controller1.GetRawAxis(2)) {
    //   // Speed Up
    //   SmartDashboard::PutString("DB/String 6", "Speed Up Motor");
    // }
    // else {
    //   SmartDashboard::PutString("DB/String 6", " ");
    // }

    if (controller1.GetRawAxis(2) && controller1.GetRawAxis(3)) {
      // Fire
      double VScale = 14.6;
      double turretShootSpeed = EstimateSpeed(EstimateDistance()) / VScale;

      if (turretShootSpeed > 1) {
        turretShootSpeed = 1.0;
      }

      turretShoot->SetSpeed(turretShootSpeed);
    }
    else if (controller1.GetRawAxis(2)) {
      // Speed Up
      turretShoot->SetSpeed(1.0);
    }
    else {
      turretShoot->SetSpeed(0.0);
    }

    // Arm Motor
    if (controller1.GetRawButton(6)) {
      // Intake
      arm->SetSpeed(1.0);
      SmartDashboard::PutString("DB/String 0", "Speed: 0.75");
    }
    else if (controller1.GetRawButton(5)) {
      // Outtake
      arm->SetSpeed(-1.0);
      SmartDashboard::PutString("DB/String 0", "Arm Speed: -0.75");
    }
    else {
      arm->SetSpeed(0.0);
      SmartDashboard::PutString("DB/String 0", "Arm Speed: 0.0");
    }

    // Turret
    if (controller1.GetRawAxis(4) >= 0.5) {
      // Input Code
      turret->SetSpeed(1.0);
    }
    else if (controller1.GetRawAxis(4) <= -0.5) {
      // Input Code
      turret->SetSpeed(-1.0);
    }
    else {
      // Input Code
      turret->SetSpeed(0.0);
    }
  }
  
  /*
  * Limelight Tracking Functions
  */
  
  void UpdateLimelightTracking() {

    // Proportional Steering Constant:
    // If robot doesn't turn fast enough, make larger
    // If robot oscillates, make smaller
    const double steerConst = 0.05;

    // Proportional Drive Constant:
    // Bigger = faster drive
    const double driveConst = 0.26;

    // Area of the target when robot reaches the goal
    const double desiredTargetArea = 13.0;
    const double maxDrive = 0.65;
    const double maxSteer = 1.0f;

    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double tx = table->GetNumber("tx", 0.0);
   // double ta = table->GetNumber("ta", 0.0);
    double tv = table->GetNumber("tv", 0.0);

    if (tv < 1.0) {
      LimelightHasTarget = false;
      LimelightDriveCmd = 0.0;
      LimelightTurnCmd = 0.0;
    }
    else {
      LimelightHasTarget = true;
      LimelightTurnCmd = tx*steerConst;
      LimelightTurnCmd = clamp(LimelightTurnCmd,-maxSteer,maxSteer);
      // LimelightDriveCmd = (desiredTargetArea - ta) * driveConst;
      // LimelightDriveCmd = clamp(LimelightDriveCmd,-maxDrive,maxDrive);
      
      // Auton section
      if (autonIsActive) {
        double VScale = 14.6;      // sets scale of max speed for PWM
        double turretShootSpeed = EstimateSpeed(EstimateDistance())/VScale;

        if (turretShootSpeed > 1) { 
          turretShootSpeed= 1.0; 
        }

        turretShoot->SetSpeed(turretShootSpeed);
        //INPUT ANGLE CODE HERE USING ESTIMATE ANGLE
      }
    }
  }

  void AutonLimelightTracking() {
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double tx = table->GetNumber("tx", 0.0);

    const double steerConst = 0.5;
    const double maxSteer = 1.0f;

    LimelightTurnCmd = tx*steerConst;
    LimelightTurnCmd = clamp(LimelightTurnCmd, -maxSteer, maxSteer);
  }

  
  double EstimateDistance() {
      std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
      double ty = table->GetNumber("ty", 0.0);
      double h2 = 20; // Height of target = 98.25
      double h1 = 26.13; // Height of camera from floor = 26.13
      double a1 = 3;  // Yaw of camera = 0
      double d = (h2 - h1) / tan(a1 + ty);
      return d;
      //D RETURNS IN INCHES:  That's ridiculous!!! Use Metric!
      //YES D RETURNS IN INCHES: This is America! Land of the obese and home of the Big Mac
  }

  double EstimateSpeed(double d) {
    double h = 1.8;
    double theta, v;
    theta = atan(2 * h / d);
    v = pow(2.0 * g * d / sin(2.0 * theta), 0.5);
    return v;
  }

  double EstimateAngle(double d) {
    double h = 1.8;
    double theta;
    theta = atan(2 * h/d);
    theta = 180 * theta / pi;
    return theta;
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { 
  return StartRobot<Robot>();
}
#endif


/*
* Maps and Assignments
*/

/*

  RoboRIO Map
  +-------------------------------------+
  | LED ligth strip = 0
  | Right Drive Cim = 1
  | Left Drive Cim  = 2
  | Turret          = 3
  | Arm             = 4
  | TurretShoot     = 5
  | 
  |
  | Color Sensor = I2C Port
  +-------------------------------------+

RoboRIO Digital Pins Map
  +-------------------------------------+
  | Auton Switch 1 (IN) = D15/D16(gnd)
  | Auton Switch 2 (IN) = D19/D20(gnd)
  | LED1 (OUT) = D11/D12(gnd)
  +-------------------------------------+

  USB Map
  +-------------------------------------+
  | Right Joystick  = 0
  | Left Joystick   = 1
  | Xbox Controller = 2
  +-------------------------------------+
  
  Xbox Controller Map
  +-------------------------------------+
  | A = Spin To Green
  | B = Spin To Red
  | Y = Spin To Yellow
  | X = Spin To Blue
  |
  | L1 = Outtake
  | L2 = Upspeed On Shooter
  | L3 =
  | R1 = Intake
  | R2 = Enable Shooting / Conveyor
  | R3 =
  |
  | DPad Right = Elevator Up
  | DPad Left  =
  | DPad Up    =
  | DPad Down  = Elevator Down
  |
  | Left Joystick  =
  | Right Joystick = x - Spin Turret
  |                  y - Adjust Angle
  | 
  | Start  =
  | Select =
  +-------------------------------------+

  Joystick Map
  +-------------------------------------+
  | Right Joystick = Arcade Movement
  | Left Joystick  = 
  |
  | RIGHT STICK
  | Right Trigger     = Switch Arcade Movement
  | On-Stick Button L = Arm Position 1
  | On-Stick Button R = Arm Position 2
  | On-Stick Button M = Limelight Lock On
  |
  | Left Button 5  = 
  | Left Button 6  = 
  | Left Button 7  =
  | Left Button 8  = Limelight Zoom Default
  | Left Button 9  = Limelight Zoom 1
  | Left Button 10 = Limelight Zoom 2
  +-------------------------------------+
  
*/

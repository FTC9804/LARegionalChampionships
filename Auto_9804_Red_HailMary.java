package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by stevecox on 2-6-16 at 4:27 pm.
 *
 * Setup at edge of first full box from the center blue/red line on the red side
 * Facing the shelter BACKWARDS
 * Drive for 3.5*2sqrt(2)*12 = 118.794inches
 */

public class Auto_9804_Red_HailMary_v2 extends LinearOpMode {

  //drive motors
  DcMotor driveLeftBack;
  DcMotor driveLeftFront;
  DcMotor driveRightBack;
  DcMotor driveRightFront;
  DcMotor spin;

  double midPower;
  int targetHeading;
  double driveGain = 0.1;
  double leftPower;
  double rightPower;
  int currentHeading = 0;                     //This is a signed value
  int headingError;
  double driveSteering;
  double currentDistance;
  int currentEncCountLeft;
  int currentEncCountRight;


  double targetDistance;
  double encoderCountsPerRotation = 1440;
  double diameter = 2;                        //diameter is found from the axis of rotation until the ground
  double circumference = diameter * 3.14159265358;
  double rotations;
  double targetEncoderCounts;
  double EncErrorLeft;
  int telemetryVariable;
  int initialEncCountLeft;
  int initialEncCountRight;

  @Override
  public void runOpMode() throws InterruptedException {
    //gives name of drive motors
    driveLeftBack = hardwareMap.dcMotor.get("m5");      // 1 on red controller SN VUTK
    driveLeftFront = hardwareMap.dcMotor.get("m6");     // 2 on red
    driveRightBack = hardwareMap.dcMotor.get("m1");     // 1 on purple controller SN UVQF
    driveRightFront = hardwareMap.dcMotor.get("m2");    // 2 on purple
    spin = hardwareMap.dcMotor.get("m8");

    ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

    hardwareMap.logDevices();

    gyro.calibrate();

    waitForStart();

    // make sure the gyro is calibrated.
    while (gyro.isCalibrating()) {
      Thread.sleep(50);
    }
    driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
    driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
    driveRightBack.setDirection(DcMotor.Direction.REVERSE);
    driveRightFront.setDirection(DcMotor.Direction.REVERSE);



    //DRIVE BACKWARDS 3.5*2sqrt2*12 = 118.794 INCHES


    telemetry.clearData();

    driveGain = 0.05;

    midPower = 0.66;
    targetHeading = 0;              //drive straight ahead

    targetDistance = 118.794;          //drive straight 95 inches
    rotations = targetDistance / circumference;
    targetEncoderCounts = encoderCountsPerRotation * rotations;

    this.resetStartTime();

    initialEncCountLeft = Math.abs(driveLeftBack.getCurrentPosition());
    initialEncCountRight = Math.abs(driveRightBack.getCurrentPosition());


    do {
      spin.setPower(1);  // Eject debris while driving , to clear path

      currentEncCountLeft = Math.abs(driveLeftBack.getCurrentPosition()) - initialEncCountLeft;
      currentEncCountRight = Math.abs(driveRightBack.getCurrentPosition()) - initialEncCountRight;

      EncErrorLeft = targetEncoderCounts - currentEncCountLeft;

      telemetry.addData("Left Encoder:", currentEncCountLeft);
      telemetry.addData("Right Encoder:", currentEncCountRight);

      currentDistance = (currentEncCountLeft * circumference) / encoderCountsPerRotation;
      telemetry.addData("Calculated current distance: ", currentDistance);
      // get the Z-axis heading info.
      //this is a signed heading not a basic heading
      currentHeading = gyro.getIntegratedZValue();

      headingError = targetHeading - currentHeading;

      driveSteering = headingError * driveGain;

      leftPower = midPower - driveSteering;
      if (leftPower > 1.0) {                            //cuts ourselves off at 1, the maximum motor power
        leftPower = 1.0;
      }
      if (leftPower < 0.0) {                            //don't drive backwards
        leftPower = 0.0;
      }
      rightPower = midPower + driveSteering;
      if (rightPower > 1.0) {
        rightPower = 1.0;
      }
      if (rightPower < 0.0) {
        rightPower = 0.0;
      }
      //when driving backwards, reverse leading and trailing
      //left front is now trailing, left back is now leading
      //trailing gets full power
      driveLeftFront.setPower(-leftPower);
      driveLeftBack.setPower(-.95 * leftPower);       //creates belt tension between the drive pulleys
      driveRightFront.setPower(-rightPower);
      driveRightBack.setPower(-.95 * rightPower);

      waitOneFullHardwareCycle();


    } while (EncErrorLeft > 0
            && this.getRuntime() < 200);

    driveLeftBack.setPower(0.0);
    driveLeftFront.setPower(0.0);
    driveRightBack.setPower(0.0);
    driveRightFront.setPower(0.0);
    telemetry.clearData();

    spin.setPower (0);

    this.resetStartTime();


    telemetry.addData("straight done", telemetryVariable);
    resetStartTime();
    while (this.getRuntime() < 1) {
      waitOneFullHardwareCycle();
    }

  }
}

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Created by stevecox on 2-24-16 at 3:47 pm.
 * Setup at back left edge of first full box from the center blue/red line on the red side
 * Facing the shelter BACKWARDS
 * Drive for 2*2sqrt(2)*12 = 67.88 inches backwards
 * spins clockwise 45º
 * Drive backwards 24 inches
 * Do all this with spin motors running
 *headings increase with counter-clockwise rotation
 */

public class Auto_9804_Straight_SlightR_Straight extends LinearOpMode {

    //drive motors
    DcMotor driveLeftBack;
    DcMotor driveLeftFront;
    DcMotor driveRightBack;
    DcMotor driveRightFront;
    DcMotor spin;

    //servos to lock in place on ramp
    Servo grabLeft;
    Servo grabRight;

    Servo box;

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
    double diameter = 2;
    double circumference = diameter * 3.14159;
    double rotations;
    double targetEncoderCounts;
    double targetEncoderCounts1;
    double targetEncoderCounts2;
    double EncErrorLeft;
    int telemetryVariable;
    int initialEncCountLeft;
    int initialEncCountRight;


    //servo variables for grab servos
    double grabLeftUp = 0;                  //0 is max CCW (UP on left side)
    double grabLeftDown = 0.6;              //0.6 is approx. 90 degrees CW (DOWN on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double grabRightDown = 0.4;             //0.4 is approx. 90 degrees CCW (DOWN on right side)
    double boxPosition = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        //gives name of drive motors
        driveLeftBack = hardwareMap.dcMotor.get("m5");      // 1 on red controller SN VUTK
        driveLeftFront = hardwareMap.dcMotor.get("m6");     // 2 on red
        driveRightBack = hardwareMap.dcMotor.get("m1");     // 1 on purple controller SN UVQF
        driveRightFront = hardwareMap.dcMotor.get("m2");    // 2 on purple
        spin = hardwareMap.dcMotor.get("m8");

        //give the configuration file names for the servos
        grabLeft = hardwareMap.servo.get("s1");             // xx on servo controller SN VSI1
        grabRight = hardwareMap.servo.get("s2");            // xx on servo controller
        box = hardwareMap.servo.get("s4");


        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        box.setPosition(boxPosition);


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



        //DRIVE BACKWARDS 67.88 INCHES


        telemetry.clearData();

        driveGain = 0.05;

        midPower = 0.66;
        targetHeading = 0;              //drive straight ahead

        targetDistance = 67.88;          //drive straight 67.88 inches
        rotations = targetDistance / circumference;
        targetEncoderCounts = encoderCountsPerRotation * rotations;
        targetEncoderCounts1 = targetEncoderCounts;

        this.resetStartTime();

        initialEncCountLeft = Math.abs(driveLeftBack.getCurrentPosition());
        initialEncCountRight = Math.abs(driveRightBack.getCurrentPosition());


        do {
            spin.setPower(1);  // Eject debris while driving , to clear path

            currentEncCountLeft = Math.abs(driveLeftBack.getCurrentPosition()) - initialEncCountLeft;
            currentEncCountRight = Math.abs(driveRightBack.getCurrentPosition()) - initialEncCountRight;

            EncErrorLeft = targetEncoderCounts1 - currentEncCountLeft;

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


        telemetry.addData("straight 1 done", telemetryVariable);
        resetStartTime();
        while (this.getRuntime() < 1) {
            waitOneFullHardwareCycle();
        }



        // SPIN CW 45 DEGREES


        driveGain = 0.05;                    //OK for spin

        midPower = 0.0;                     //spin move: zero driving-forward power
        targetHeading = -45;                // 45 degrees CW (using signed heading)

        this.resetStartTime();

        do {

            // get the Z-axis heading info.
            // this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            telemetry.addData("Current Angle: ", currentHeading);

            headingError = targetHeading - currentHeading;

            driveSteering = headingError * driveGain;

            leftPower = midPower - driveSteering;
            if (leftPower > 1.0) {                            //cuts ourselves off at 1, the maximum motor power
                leftPower = 1.0;
            }
            if (leftPower < -1) {                            //treads always moving forward
                leftPower = -1;
            }
            rightPower = midPower + driveSteering;
            if (rightPower > 1.0) {
                rightPower = 1.0;
            }
            if (rightPower < -1) {
                rightPower = -1;
            }
            //when drving backwards, reverse leading and trailing
            //left front is now trailing, left back is now leading
            //trailing gets full power
            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(rightPower);
            driveRightBack.setPower(.95 * rightPower);

            waitOneFullHardwareCycle();


        } while (currentHeading > targetHeading         //we are going to -45, so we will loop while >
                && this.getRuntime() < 2);

        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);

        telemetry.addData("spin 45 done", telemetryVariable);
        resetStartTime();
        while (this.getRuntime() < 1) {
            waitOneFullHardwareCycle();
        }



        //DRIVE BACKWARDS 24 INCHES


        telemetry.clearData();

        driveGain = 0.05;

        midPower = 0.75;
        targetHeading = -45;              //drive straight ahead

        targetDistance = 24.0;          //drive straight 24 inches
        rotations = targetDistance / circumference;
        targetEncoderCounts = encoderCountsPerRotation * rotations;
        targetEncoderCounts2 = targetEncoderCounts1 - targetEncoderCounts;

        this.resetStartTime();

        initialEncCountLeft = Math.abs(driveLeftBack.getCurrentPosition());
        initialEncCountRight = Math.abs(driveRightBack.getCurrentPosition());


        do {
            spin.setPower(1);  // Eject debris while driving

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

        grabRight.setPosition(grabRightDown);
        grabLeft.setPosition(grabLeftDown);

        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        telemetry.clearData();

        spin.setPower (0);

        this.resetStartTime();


        telemetry.addData("straight 2 done", telemetryVariable);
        resetStartTime();
        while (this.getRuntime() < 1) {
            waitOneFullHardwareCycle();
        }

        telemetry.addData("CODE COMPLETE", telemetryVariable);
        resetStartTime();
        while (this.getRuntime() < 1) {
            waitOneFullHardwareCycle();
        }



    }
}

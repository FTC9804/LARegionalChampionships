package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by stevecox on 2-6-16 at 4:50 pm.
 * Setup at edge of second full box from the center blue/red line on the red side
 * Facing the shelter BACKWARDS
 * Drive for 3*2sqrt(2)*12 = 101.823 inches
 * spins clockwise 45º
 * Drive forward 24 inches
 */
public class Auto_9804_Red_TheJuke extends LinearOpMode {

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
    double diameter = 2;
    double circumference = diameter * 3.14159;
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



        //DRIVE BACKWARDS 110 INCHES


        telemetry.clearData();

        driveGain = 0.05;

        midPower = 0.66;
        targetHeading = 0;              //drive straight ahead

        targetDistance = 101.823;          //drive straight 95 inches
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
            //when drving backwards, reverse leading and trailing
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



        // SPIN LEFT 90 DEGREES


        driveGain = 0.05;                    //OK for spin

        midPower = 0.0;                     //spin move: zero driving-forward power
        targetHeading = 45;                // 90 degrees CW (using signed heading)

        telemetry.clearData();

        this.resetStartTime();

        do {
            spin.setPower(1);  // Eject debris while driving , to clear path

            // get the Z-axis heading info.
            // this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

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
            //when driving backwards, reverse leading and trailing
            //left front is now trailing, left back is now leading
            //trailing gets full power
            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(rightPower);
            driveRightBack.setPower(.95 * rightPower);

            waitOneFullHardwareCycle();


        } while (currentHeading < targetHeading         //we are going to +90, so we will loop while <
                && this.getRuntime() < 2);

        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);

        telemetry.addData("spin done", telemetryVariable);
        resetStartTime();
        while (this.getRuntime() < 1) {
            waitOneFullHardwareCycle();
        }



        //DRIVE BACKWARDS 120 INCHES


        telemetry.clearData();

        driveGain = 0.05;

        midPower = 0.75;
        targetHeading = 45;              //drive straight ahead

        targetDistance = 24;          //drive straight 120 inches
        rotations = targetDistance / circumference;
        targetEncoderCounts = encoderCountsPerRotation * rotations;

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


    }
}

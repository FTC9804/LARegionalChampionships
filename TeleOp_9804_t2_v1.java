package com.qualcomm.ftcrobotcontroller.opmodes;

//import OpModes

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

// Made by the programmers of FTC Team 9804 Bomb Squad on January 27, 2016
//V02 6 pm 1-28-16
// renamed to TeleOp_9804_v1 (for Qualifier!) on 1-29-16 at 12:37 PM
//renamed to TeleOp_9804_v2 on 1-30-16 at 9:00 AM
//renamed to TeleOP_9804_v3 on 1-30-16 at 3:31 PM
// Steve C.


public class TeleOp_9804_v4_variableGain extends OpMode {

    //defining the motors, servos, and variables in this program

    //magnetic sensor
    DigitalChannel sensorExtend;        // detects magnet, arms fully extended
    DigitalChannel sensorRetract;       // detects magnet, arms fully retracted
    DigitalChannel extendLED;           // these are indicator LEDs
    DigitalChannel retractLED;          // that will signal the drivers when arm limits are reached

    //drive motors
    DcMotor driveLeftBack;
    DcMotor driveLeftFront;
    DcMotor driveRightBack;
    DcMotor driveRightFront;

    //winch motors
    DcMotor leftWinch;
    DcMotor rightWinch;

    //motors for extending arms and spinner (debris collector)
    DcMotor arms;
    DcMotor spin;

    //servos to lock in place on ramp
    Servo grabLeft;
    Servo grabRight;

    //servo to release debris
    Servo score;

    //servo to drop arm for climbers
    Servo drop;
    //variables for driving
    float trailingPowerRight;
    double leadingPowerRight;
    float trailingPowerLeft;
    double leadingPowerLeft;

    //servo variables for grab servos
    double grabLeftUp = 0;                  //0 is max CCW (UP on left side)
    double grabLeftDown = 0.6;              //0.6 is approx. 90 degrees CW (DOWN on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double grabRightDown = 0.4;             //0.4 is approx. 90 degrees CCW (DOWN on right side)

    //servo variables for score servo
    double scoreClosed = 1.0;               //hopper door is closed (UP)
    double scoreOpened = 0.0;               //hopper door is open (DOWN)
    double scorePosition = scoreClosed;     //current position of door, initialize to UP

    //servo variable for continuous rotation drop servo
    double dropMovingDown = 0.75;
    double dropMovingUp = 0.25;
    double dropStopMoving = 0.5;

    //gives the state of the magnet sensors for the LED activation and ability to stop the motors
    boolean armsNotExtended = true; // state of magnetic sensors
    boolean armsNotRetracted = true;

    //variables for the winch motors to allow automatic control with manual override
    double leftWinchSpeed = 0;
    double rightWinchSpeed = 0;

    float joystickGainR = 1;
    float joystickGainL = 1;

    float joystick1ValueRight;
    float joystick1ValueLeft;

    @Override
    public void init() {

        //NAMES FOR CONFIGURATION FILES ON ZTE PHONES

        //gives name of magnetic sensors and LEDs for the configuration files
        sensorExtend = hardwareMap.digitalChannel.get("mag1");
        sensorRetract = hardwareMap.digitalChannel.get("mag2");
        extendLED = hardwareMap.digitalChannel.get("led1");
        retractLED = hardwareMap.digitalChannel.get("led2");
        extendLED.setMode(DigitalChannelController.Mode.OUTPUT);//the LEDs will be given a logical
        retractLED.setMode(DigitalChannelController.Mode.OUTPUT);//output signal to turn on/off
        retractLED.setState(false);                   // LEDs initialized "off"
        extendLED.setState(false);

        //gives name of drive motors
        driveLeftBack = hardwareMap.dcMotor.get("m5");      // 1 on red controller SN VUTK
        driveLeftFront = hardwareMap.dcMotor.get("m6");     // 2 on red

        driveRightBack = hardwareMap.dcMotor.get("m1");     // 1 on purple controller SN UVQF
        driveRightFront = hardwareMap.dcMotor.get("m2");    // 2 on purple

        // set direction of L and R drive motors, since they are opposite-facing
        driveRightFront.setDirection(DcMotor.Direction.FORWARD);  // right side forward
        driveRightBack.setDirection(DcMotor.Direction.FORWARD);   // with positive voltage
        driveLeftBack.setDirection(DcMotor.Direction.REVERSE);    // so we reverse the left side
        driveLeftFront.setDirection(DcMotor.Direction.REVERSE);

        //gives motor names for the other motors
        arms = hardwareMap.dcMotor.get("m7");               // 1 on green controller SN VF7F
        spin = hardwareMap.dcMotor.get("m8");              // 2 on green

        //gives names of winch motors in the configuration files
        leftWinch = hardwareMap.dcMotor.get("m4");         // 1 on orange controller SN XTJI
        rightWinch = hardwareMap.dcMotor.get("m3");        // 2 on orange

        //give the servo names for the servos
        grabLeft = hardwareMap.servo.get("s1");             // xx on servo controller SN VSI1
        grabRight = hardwareMap.servo.get("s2");            // xx on servo controller
        score = hardwareMap.servo.get("s3");
        drop = hardwareMap.servo.get("s4");

        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        score.setPosition(scoreClosed);
        drop.setPosition(dropStopMoving);

        this.resetStartTime();     //reset to allow time for servos to reach initialized positions

        while (this.getRuntime() < 1) {

        }

        //reset timer for match
        this.resetStartTime();
    }

    @Override
    public void loop() {
        //creates boolean value for magnetic sensor,
        //true = no magnet detected nearby
        //false = have reached limit
        armsNotExtended = sensorExtend.getState();
        armsNotRetracted = sensorRetract.getState();
        //telemetry for magnetic sensors on the driver station
        telemetry.addData("Extended Sensor", String.format("%1d", (armsNotExtended ? 1 : 0)));
        telemetry.addData("Retracted Sensor", String.format("%1d", (armsNotRetracted ? 1 : 0)));

        //takes input from joysticks for motor values;
        // sets the front wheel at a lesser power to ensure belt tension

        joystick1ValueLeft = gamepad1.left_stick_y;
        joystick1ValueRight = gamepad1.right_stick_y;

        if (Math.abs(joystick1ValueLeft) >= 0.1 && Math.abs(joystick1ValueLeft) < 0.4){
            joystickGainL = (float)0.4;
        } else if (Math.abs(joystick1ValueLeft) >= 0.4 && Math.abs(joystick1ValueLeft) < 0.7){
            joystickGainL = (float)0.7;
        } else{
            joystickGainL = (float)1;
        }
        if (Math.abs(joystick1ValueRight) >= 0.1 && Math.abs(joystick1ValueRight) < 0.4){
            joystickGainR = (float)0.4;
        } else if (Math.abs(joystick1ValueRight) >= 0.4 && Math.abs(joystick1ValueRight) < 0.7){
            joystickGainR = (float)0.7;
        } else{
            joystickGainR = (float)1;
        }
        trailingPowerLeft = joystick1ValueLeft * joystickGainL;
        leadingPowerLeft = .95 * trailingPowerLeft;
        trailingPowerRight = joystick1ValueRight * joystickGainR;
        leadingPowerRight = .95 * trailingPowerRight;


        driveLeftBack.setPower(trailingPowerLeft);
        driveRightBack.setPower(trailingPowerRight);

        if (leadingPowerLeft > 0.1) {                       // ignore dead zone on joystick
            driveLeftBack.setPower(trailingPowerLeft);      //dead zone is the area right next to 0,
            driveLeftFront.setPower(leadingPowerLeft);      //but not 0, where the motors are still
        } else if (leadingPowerLeft < -0.1) {               //straining to run
            driveLeftBack.setPower(leadingPowerLeft);
            driveLeftFront.setPower(trailingPowerLeft);
        } else {
            driveLeftFront.setPower(0);
            driveLeftBack.setPower(0);
        }
        if (leadingPowerRight > 0.1) {
            driveRightBack.setPower(trailingPowerRight);
            driveRightFront.setPower(leadingPowerRight);
        } else if (leadingPowerRight < -0.1) {
            driveRightBack.setPower(leadingPowerRight);
            driveRightFront.setPower(trailingPowerRight);
        } else {
            driveRightBack.setPower(0);
            driveRightFront.setPower(0);
        }


        if (armsNotRetracted) {                 //set states of LED based on the positions of
            retractLED.setState(false);         //the magnet sensor and magnet
        } else {
            retractLED.setState(true);
        }
        if (armsNotExtended) {
            extendLED.setState(false);
        } else {
            extendLED.setState(true);
        }


        if ((gamepad2.right_bumper || gamepad2.left_bumper) && armsNotExtended) {      //moves arms and winches with d-pad buttons,
            arms.setPower(-1);                          //as long as the magnetic sensor has not
        } else if ((gamepad2.right_trigger > .3 || gamepad2.left_trigger > .3) && armsNotRetracted) {
            arms.setPower(1);
        } else {
            arms.setPower(0);
        }

        if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {    //allow manual
            leftWinchSpeed = gamepad2.left_stick_y;                         //override of the winch
        }                                                                   //motors when driver
        if (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1) {  //wants control
            rightWinchSpeed = gamepad2.right_stick_y;
        }
        else {
            leftWinchSpeed = 0;
            rightWinchSpeed = 0;
        }

        rightWinch.setPower(rightWinchSpeed);           //sets power of the winches to the
        leftWinch.setPower(leftWinchSpeed);             //specified power


        //takes input from buttons for spin motors
        if (gamepad2.a) {                   //collect debris
            spin.setPower(-1);
        } else if (gamepad2.y) {            //eject or sweep away debris
            spin.setPower(1);
        } else {
            spin.setPower(0);
        }


        //takes input from bumpers and triggers for the locking grab motors set individually
        if (gamepad1.right_bumper) {
            grabRight.setPosition(grabRightUp);
        } else if (gamepad1.right_trigger > .3) {       //these triggers are considered axis,
            grabRight.setPosition(grabRightDown);       //but we effectively utilize them as
        }                                               //buttons by using them like this


        if (gamepad1.left_bumper) {
            grabLeft.setPosition(grabLeftUp);
        } else if (gamepad1.left_trigger > .3) {
            grabLeft.setPosition(grabLeftDown);
        }

        //sets score to a value because a button is pressed
        if (gamepad1.a) {                           //when button is pressed, set the position to
            scorePosition -= .02;                   //just less than it was in the previous loop
            if (scorePosition < scoreOpened) {      //when the score position's math tells it to go
                scorePosition = scoreOpened;        //beyond the maximum point,
            }                                       //set it to the maximum point
        } else if (gamepad1.y) {                    //when pressing the button,
            scorePosition = scoreClosed;            //return to starting position
        }
        score.setPosition(scorePosition);           //set the servo to the position designated

        //moves drop servo to where the driver desires, continuous rotation so
        //1 is full speed CW, 0.5 is stop, 0 is full speed CCW
        if (gamepad1.dpad_up) {
            drop.setPosition(dropMovingUp);
        } else if (gamepad1.dpad_down) {
            drop.setPosition(dropMovingDown);
        } else {
            drop.setPosition(dropStopMoving);
        }


    }//finish loop
}//finish program

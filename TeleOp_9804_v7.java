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
//renamed to TeleOp_9804_v4_variableGain on 2-14-16 at 9:00 PM Etienne L.
//first draft of controlling arms extension relative to winching speed
//renamed to TeleOp_9804_v5 on 2-19-16 at 4:44 PM Steve C. 
//continue coding cleanup and commenting, setup buttons for arms & winch extension
//added clipping for the gain
//renamed to TeleOp_9804_v6 on 2-19-16 at 8:14 PM by Steve C. & Etienne L.
//establish buttons to use for competition -- includes sweep and box servos
//renamed to TeleOp_9804_v7 on 2-20-16 at 4:16 PM by Steve C.
//get code ready for driver testing and practice field



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
    Servo box;
    
    //servo for debris sweeping away
    Servo sweep;
    
    //variables for driving
    double trailingPowerRight;				//this code allows us to always give slightly
    double leadingPowerRight;				//less power to leading motor to always
    double trailingPowerLeft;				//ensure tension between the treads and 
    double leadingPowerLeft;				//ground for maximum driver control

    //servo variables for grab servos
    double grabLeftUp = 0;                  //0 is max CCW (UP on left side)
    double grabLeftDown = 0.6;              //0.6 is approx. 90 degrees CW (DOWN on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double grabRightDown = 0.4;             //0.4 is approx. 90 degrees CCW (DOWN on right side)

    //servo variables for score servo
    double scoreClosed = 1.0;               //hopper door is closed (UP)
    double scoreOpened = 0.0;               //hopper door is open (DOWN)
    double scorePosition = scoreClosed;     //current position of door, initialize to UP

    //servo variable for continuous rotation box servo
    double boxMovingDown = 0.8;
    double boxMovingUp = 0.2;
    double boxStopMoving = 0.5;
    double boxPower = 0.5;
    
    double sweepOpened = 0.5;
    double sweepClosed = 0;
    double sweepPosition = sweepClosed;
    

    //gives the state of the magnet sensors for the LED activation and ability to stop the motors
    boolean armsNotExtended = false; 	// state of magnetic sensors to false to light up
    boolean armsNotRetracted = false;	//for initialization sequence

    //variables for the winch motors to allow automatic control with manual override
    double leftWinchSpeed = 0;
    double rightWinchSpeed = 0;

    float joystickGainR = 1;
    float joystickGainL = 1;

    float joystick1ValueRight;
    float joystick1ValueLeft;
    
    //these are declarations for proportional winch to arm power
	double initialWinchPosition;			//initial position of the winch
	double currentWinchEncoderCounts;		//encoder counts for the winch motor
	double initialArmPosition;				//initial position of the arms
	double currentArmEncoderCounts;			//encoder counts for the arms motor
	double armsSpeedGain = 1.744;			//need better estimate
	double currentWinchSpeed;				//calculated speed for winch
	double currentArmsSpeed;				//calculated speed for arms
	double targetArmsSpeed;					//desired speed for arms
	double armsSpeedError;					//error between target and current speeds
	double armsMotorPower;					//power giving to the arms
	double winchCircumference;				//circumference of the winch
	double winchDiameter = 2.5; 			//inches
	double armsPinionDiameter = 0.75; 		//inches
	double pinionCircumference;				//circumference of pinion
	double winchToArmRatio;					//ratio of winch to arm
	double armsInchesPerRotation;			//inches per rotation of arms
	double winchInchesPerRotation;			//inches per rotation of winches

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
        box = hardwareMap.servo.get("s4");
        
        //calculate the winch to arm ratio
		getWinchToArmRatio(winchDiameter, armsPinionDiameter);

        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        score.setPosition(scoreClosed);
        box.setPosition(boxStopMoving);

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



        //takes input from buttons for spin motors
        if (gamepad2.a) {                   //collect debris
            spin.setPower(-1);
            boxPower = boxMovingUp;
        } else if (gamepad2.y) {            //eject or sweep away debris
            spin.setPower(1);
            boxPower = boxStopMoving;
        } else {
            spin.setPower(0);
            boxPower = boxStopMoving;
        }

		if (gamepad1.y) {
			boxPower = boxMovingDown;
		}
		else if (gamepad1.a) {
			boxPower = boxMovingUp;
		}
		else {
			boxPower = boxStopMoving;
		}

		box.setPosition(boxPower); 
		
		
		if (gamepad1.b) {
			sweepPosition = sweepOpened;
		}
		else {
			sweepPosition = sweepClosed;
		}
		
		sweep.setPosition(sweepPosition);

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


    //Proportional Arm and Winch extension

		if ((gamepad2.dpad_up || gamepad1.dpad_up) && armsNotExtended) {      //moves arms and winches with d-pad buttons,

			leftWinch.setPower(1.0);
			rightWinch.setPower(1.0);
			this.resetStartTime();

			
			initialWinchPosition = Math.abs(leftWinch.getCurrentPosition());
			initialArmPosition = Math.abs(arms.getCurrentPosition());

			while (this.getRuntime() < 50) {} //assuming 50 is in ms

			currentWinchEncoderCounts = Math.abs(leftWinch.getCurrentPosition()) - initialWinchPosition;
			currentArmEncoderCounts = Math.abs(arms.getCurrentPosition()) - initialArmPosition;

			//calculate the speeds
			currentWinchSpeed = currentWinchEncoderCounts/this.time; //units are clicks / ms
			currentArmsSpeed = currentArmEncoderCounts/this.time; 	//units are clicks / ms

			targetArmsSpeed = currentWinchSpeed * winchToArmRatio;

			armsSpeedError =  currentArmsSpeed - targetArmsSpeed;

			armsMotorPower = armsSpeedError * armsSpeedGain;

			if (armsMotorPower > 1) {
				armsMotorPower = 1;
			}
			if (armsMotorPower < 0) {
				armsMotorPower = 0;
			}


			arms.setPower(armsMotorPower);

        } else if ((gamepad2.dpad_down || gamepad1.dpad_down) && armsNotRetracted) {

			leftWinch.setPower(-1.0);
			rightWinch.setPower(-1.0);
			this.resetStartTime();

			//
			initialWinchPosition = Math.abs(leftWinch.getCurrentPosition());
			initialArmPosition = Math.abs(arms.getCurrentPosition());
		

			while (this.getRuntime() < 50) {} //assuming 50 is in ms

			currentWinchEncoderCounts = Math.abs(leftWinch.getCurrentPosition()) - initialWinchPosition;
			currentArmEncoderCounts = Math.abs(arms.getCurrentPosition()) - initialArmPosition;

			//calculate the speeds
			currentWinchSpeed = currentWinchEncoderCounts/this.time; //units are clicks / ms
			currentArmsSpeed = currentArmEncoderCounts/this.time; 	//units are clicks / ms

			targetArmsSpeed = currentWinchSpeed * winchToArmRatio;

			armsSpeedError =  currentArmsSpeed - targetArmsSpeed;

			armsMotorPower = armsSpeedError * armsSpeedGain;

			if (armsMotorPower < -1) {
				armsMotorPower = -1;
			}
			if (armsMotorPower > 0) {
				armsMotorPower = 0;
			}

			arms.setPower(armsMotorPower);

        } else {
            arms.setPower(0);
			rightWinchSpeed = 0;
			leftWinchSpeed = 0;
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



	}//finish loop

	public double getWinchToArmRatio(double winchDia, double armsDia){

		winchCircumference = winchDia * 3.14159;

		winchInchesPerRotation = (winchCircumference )/4; //Gear ratio is 4

		pinionCircumference = armsDia * 3.14159;

		armsInchesPerRotation = pinionCircumference; //  gear ratio is one

		winchToArmRatio = winchInchesPerRotation/armsInchesPerRotation;

		telemetry.addData("1", "winch to arm ratio:  " + String.format("%.2f", winchToArmRatio));

		return winchToArmRatio;
	}//finish ratio calculator
	
}//finish program

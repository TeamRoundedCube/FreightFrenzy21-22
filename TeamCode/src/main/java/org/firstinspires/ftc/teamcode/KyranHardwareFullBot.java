//  ____          ____  ____           ____   ______________                     ______            ____        ____
// |    |       /    /  \   \        /    /  |     _______   \                 /        \         |     \     |   |
// |    |     /    /     \   \     /    /    |   |        |   \              /    / \    \        |      \    |   |
// |    |   /    /        \   \  /    /      |   |_______|    /            /    /    \    \       |       \   |   |
// |    | /    /           \   \/   /        |     ___      _/           /    /       \    \      |    |\  \  |   |
// |    | \    \            |     |          |    |   \    \           /    /__________\    \     |    | \  \ |   |
// |    |  \    \           |     |          |    |    \    \        /    /_____________\    \    |    |  \       |
// |    |   \    \          |     |          |    |     \    \     /    /                \    \   |    |   \      |
// |____|    \____\         |_____|          |____|      \____\  /____/                   \____\  |____|    \_____|


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//Created by Kyran 10/16/2021 @ 3:00pm
//Purpose: Used For Marc to test Teleop ("MarcsTeleop") on the Robot

public class KyranHardwareFullBot {

    //Objects
    public DcMotorEx front_left  =null;
    public DcMotorEx front_right = null;
    public DcMotorEx back_left =null;
    public DcMotorEx back_right =null;
    public DcMotor spincarousel = null;
    public DcMotor arm;
    public DcMotor intake;
    //public DcMotorEx shooterEx;
    //public WebcamName webcam;
    //public VoltageSensor vsense;

    //public ColorSensor frontColor;
    //public ColorSensor color_right;
    //public ColorSensor color_left;
    //public DistanceSensor distance_left;
    //public ColorSensor bottomColor;

    public Servo basket;
    //public Servo flick;
    //public Servo drop;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public KyranHardwareFullBot()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {     //Method
        // Save reference to Hardware map
        hwMap=ahwMap;
        //webcam = hwMap.get(WebcamName.class, "Webcam");

        // Define and Initialize Motors
 /*
<Motor name="front_left" port="0"/>
<Motor name="front_right" port="1"/>
<Motor name="back_left" port="2"/>
<Motor name="back_right" port="3"/>
 */
        front_left  = hwMap.get(DcMotorEx.class, "front_left");
        front_right = hwMap.get(DcMotorEx.class, "front_right");
        back_left   = hwMap.get(DcMotorEx.class, "back_left");
        back_right  = hwMap.get(DcMotorEx.class, "back_right");

        front_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        front_right.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        back_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        back_right .setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        PIDFCoefficients p; //We are not using this?

        intake = hwMap.get(DcMotor.class, "intake");
        arm = hwMap.get(DcMotor.class, "arm");
        spincarousel = hwMap.get(DcMotor.class, "spin_carousel");

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //frontColor = hwMap.get(ColorSensor.class,"frontcolor");
        //bottomColor = hwMap.get(ColorSensor.class,"bottomcolor");

        // Define and initialize ALL installed servos.
   //     flick = hwMap.get(Servo.class, "flick");
   //     drop = hwMap.get(Servo.class, "drop");
        basket = hwMap.get(Servo.class, "basket");
/*
        color_left = hwMap.get(ColorSensor.class, "color_left");
        color_right = hwMap.get(ColorSensor.class, "color_right");
        distance_left = hwMap.get(DistanceSensor.class, "distance_left");

        frontColor.green();
    */

    }
}




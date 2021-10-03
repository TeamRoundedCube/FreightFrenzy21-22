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

//edit by Prabhav 9/26/21 @ 9:39am
//Comment added by Rushda to test on 9-25-21 at 7:34PM to test if push and commit works fine or not
// Comment added by Rajiv J 9/26/2021 at 9:44am
public class HardwareFullBot {
    /* Public OpMode members. */
    //Created Objects
    public DcMotorEx front_left  =null;
    public DcMotorEx front_right = null;
    public DcMotorEx back_left =null;
    public DcMotorEx back_right =null;
    public DcMotor spincarousel = null;
    //public DcMotorEx shooterEx;
    //public DcMotor shooter;
    //public DcMotor arm;
    //public DcMotor intake;
    //public WebcamName webcam;
    //public VoltageSensor vsense;

    //public ColorSensor frontColor;
    // public ColorSensor color_right;
    //public ColorSensor color_left;
    //public DistanceSensor distance_left;
    //  public ColorSensor bottomColor;



    //public Servo claw;
    //public Servo flick;
    //public Servo drop;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareFullBot(){


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
        back_left    = hwMap.get(DcMotorEx.class, "back_left");
        back_right    = hwMap.get(DcMotorEx.class, "back_right");

        front_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        front_right.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        back_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        back_right .setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        PIDFCoefficients p;





        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //frontColor = hwMap.get(ColorSensor.class,"frontcolor");
        //     bottomColor = hwMap.get(ColorSensor.class,"bottomcolor");
        /*intake = hwMap.get(DcMotor.class, "intake");
        arm = hwMap.get(DcMotor.class, "arm");
        shooter = hwMap.get(DcMotor.class, "shooter");
        shooterEx = hwMap.get(DcMotorEx.class, "shooter");

        // Define and initialize ALL installed servos.
        flick = hwMap.get(Servo.class, "flick");
        drop = hwMap.get(Servo.class, "drop");
        claw = hwMap.get(Servo.class, "claw");

        color_left = hwMap.get(ColorSensor.class, "color_left");
        color_right = hwMap.get(ColorSensor.class, "color_right");
        distance_left = hwMap.get(DistanceSensor.class, "distance_left");


    */
        //frontColor.green();

    }
}




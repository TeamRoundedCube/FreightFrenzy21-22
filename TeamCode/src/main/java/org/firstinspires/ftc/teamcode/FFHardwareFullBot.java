//  ____      ____      ____           ____   ______________              _______            ____        ___
// |    |    /    /     \   \        /    /  |     _______   \          /         \         |     \     |   |
// |    |   /    /       \   \     /    /    |   |        |   \        /    / \    \        |      \    |   |
// |    |  /    /         \   \  /    /      |   |_______|    /       /    /   \    \       |       \   |   |
// |    | /    /           \   \/   /        |     ___      _/       /    /     \    \      |    |\  \  |   |
// |    | \    \            |     |          |    |   \    \        /    /_______\    \     |    | \  \ |   |
// |    |  \    \           |     |          |    |    \    \      /    /_________\    \    |    |  \       |
// |    |   \    \          |     |          |    |     \    \    /    /           \    \   |    |   \      |
// |____|    \____\         |_____|          |____|      \____\  /____/             \____\  |____|    \_____|


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

//Created by Kyran 10/16/2021 @ 3:00pm
//Purpose: Used For Marc to test Teleop ("MarcsTeleop") on the Robot

public class FFHardwareFullBot {

    //Objects
    public DcMotorEx front_left  = null;
    public DcMotorEx front_right = null;
    public DcMotorEx back_left = null;
    public DcMotorEx back_right = null;
    public DcMotor spincarousel = null;
    public DcMotor arm;
    public DcMotor intake;
    //public DcMotorEx shooterEx;
    public WebcamName webcam;
    //public VoltageSensor vsense;

    public GyroSensor gyro;

    public BNO055IMU imu;
    //public DistanceSensor right_bk_distance;   // port 0 on control hub
   // public DistanceSensor right_fr_distance;    //port 1 on control hub
   // public DistanceSensor left_distance;     //port 2 on control hub
    public DistanceSensor front_distance;  //port 3 on control hub

    public Servo basket;
    public Servo element;

    public RevBlinkinLedDriver led;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    Deadline ledCycleDeadline;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public FFHardwareFullBot()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {     //Method
        // Save reference to Hardware map
        hwMap=ahwMap;
        webcam = hwMap.get(WebcamName.class, "Webcam");

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

        arm.setZeroPowerBehavior(BRAKE);

        front_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define and initialize ALL installed servos.
       element = hwMap.get(Servo.class, "element");
        basket = hwMap.get(Servo.class, "basket");

        //right_bk_distance = hwMap.get(DistanceSensor.class, "right_bk_distance");
      //  right_fr_distance = hwMap.get(DistanceSensor.class, "right_fr_distance");
     //   left_distance = hwMap.get(DistanceSensor.class, "left_distance");
        front_distance = hwMap.get(DistanceSensor.class, "front_distance");
     //   frontColor.green();

        gyro = hwMap.gyroSensor.get("gyro");
        led = hwMap.get(RevBlinkinLedDriver.class, "blinkin");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }
}




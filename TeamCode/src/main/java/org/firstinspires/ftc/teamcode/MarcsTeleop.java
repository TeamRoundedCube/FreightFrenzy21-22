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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

//Created by Kyran 10/16/2021 @ 3:05pm
//Purpose: Marc will use to test the new competition robot


@TeleOp(name = "MarcsTeleop")
public class MarcsTeleop extends OpMode {

    KyranHardwareFullBot robot = new KyranHardwareFullBot();
    boolean shooting = false;
    boolean autoAim = false;
    boolean squared = false;
    boolean light = false;
    // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {

        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //Variables

        float turnPower = -gamepad1.right_stick_x; //Turn robot
        float forwardPower = -gamepad1.left_stick_y; //FOrward and Back
        float strafePower = gamepad1.left_stick_x; //Strafe
        //float diagonalPoser = ?; //Diagonal
        // /*
        telemetry.addData("leftx:", java.lang.Math.abs(gamepad1.left_stick_x));
        telemetry.addData("lefty:", java.lang.Math.abs(gamepad1.left_stick_y));
        telemetry.update();
        // */
        // Diagonal
        boolean diagonalY = (java.lang.Math.abs(gamepad1.left_stick_y) > 0.3 && java.lang.Math.abs(gamepad1.left_stick_y) < 0.8);
        boolean diagonalX = (java.lang.Math.abs(gamepad1.left_stick_x) > 0.3 && java.lang.Math.abs(gamepad1.left_stick_x) < 0.8);
        boolean diagonal = diagonalY && diagonalX;


        //Gamepad1
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//if (!diagonal) {
        // Turning (Right stick X)

        robot.front_left.setPower(-turnPower);
        robot.front_right.setPower(turnPower);
        robot.back_left.setPower(-turnPower);
        robot.back_right.setPower(turnPower);

        // Forward (Left stick Y)
        robot.front_left.setPower(forwardPower);
        robot.front_right.setPower(forwardPower);
        robot.back_left.setPower(forwardPower);
        robot.back_right.setPower(forwardPower);

        // Strafe (Left stick X)
        robot.front_left.setPower(strafePower);
        robot.front_right.setPower(-strafePower);
        robot.back_left.setPower(-strafePower);
        robot.back_right.setPower(strafePower);
//}

        // Arm Position Values
/*
        // If arm starts up
        int downPosition = 1464;
        int upPosition = 0;
        int wallPosition = 675;
*/

        //If arm starts down
        int downPosition = 0;
        int drivingPosition = 350;
        int levelOne = 3000;
        int levelTwo = 2555;
        int levelThree = 1950;

///*
        //Arm
        if (gamepad1.dpad_down) {
            robot.arm.setPower(-0.1);
            robot.arm.setTargetPosition(downPosition);//Intake position
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
        } else if (gamepad1.dpad_right) {
            robot.arm.setPower(-0.25);
            robot.arm.setTargetPosition(levelOne);// Level 1
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
        } else if (gamepad1.dpad_left) {
            robot.arm.setPower(-0.25);
            robot.arm.setTargetPosition(levelTwo);//Level 2
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
        } else if (gamepad1.dpad_up) {
            //sleep(1000);
            robot.arm.setPower(0.8);
            robot.arm.setTargetPosition(levelThree);//Level 3
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
        } else if (gamepad1.right_bumper) {

            robot.arm.setPower(0.25);
            robot.arm.setTargetPosition(drivingPosition);//Driving Position
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);

        }
//*/
        robot.spincarousel.setPower(gamepad1.right_trigger);
            //robot.spincarousel.setPower(1);
        robot.spincarousel.setPower(-gamepad1.left_trigger);
            //robot.spincarousel.setPower(-1);

        //X button Override

        if (gamepad1.x) {
            robot.arm.setPower(0);
            robot.basket.setPosition(.18);
            sleep(1000);
            //robot.flick.setPosition(0.38);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.claw.resetDeviceConfigurationForOpMode();
        }



        //Gamepad2


        // intake power
        float intakePower = -gamepad2.left_stick_y;
        robot.intake.setPower(intakePower);

        if(gamepad2.y) {
            robot.basket.setPosition(0);
            //sleep(1000);
        } else if(gamepad2.b) {
            robot.basket.setPosition(0.3);
            //sleep(1000);
        } else if (gamepad2.x) {
            robot.basket.setPosition(.18);
            //sleep(1000);
        }

        //telemetry.addData("Distance (cm)",
          //      String.format(Locale.US, "%.02f", robot.color_left.getDistance(DistanceUnit.CM)));
       /* telemetry.addData("Alpha", robot.color_left.alpha());
        telemetry.addData("Red  ", robot.color_left.red());
        telemetry.addData("Green", robot.color_left.green());
        telemetry.addData("Blue ", robot.color_left.blue());
        */
        //telemetry.addData("Hue", hsvValues[0]);


        telemetry.addData("Encoder", robot.arm.getCurrentPosition());
        telemetry.addData("Basket", robot.basket.getPosition());
        //telemetry.addData("RightTrigger", gamepad1.right_trigger);
        //telemetry.addData("autoAim?", autoAim);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.arm.setPower(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
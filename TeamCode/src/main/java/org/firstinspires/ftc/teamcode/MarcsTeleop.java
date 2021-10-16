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
        int levelOne = 3585;
        int levelTwo = 3200;
        int levelThree = 2700;

///*
        //Arm
        if (gamepad1.dpad_down) {
            robot.arm.setPower(-0.8);
            robot.arm.setTargetPosition(downPosition);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
        } else if (gamepad1.dpad_right) {
            robot.arm.setPower(-0.25);
            robot.arm.setTargetPosition(levelOne);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
        } else if (gamepad1.dpad_left) {
            robot.arm.setPower(-0.25);
            robot.arm.setTargetPosition(levelThree);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
        } else if (gamepad1.dpad_up) {
            //sleep(1000);
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setPower(0.8);
            robot.arm.setTargetPosition(levelTwo);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//*/
        robot.spincarousel.setPower(gamepad1.right_trigger);
        robot.spincarousel.setPower(-gamepad1.left_trigger);

        //X button Override

        if (gamepad1.x) {
            robot.arm.setPower(0);
            //robot.drop.setPosition(1);
            //robot.flick.setPosition(0.38);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.claw.resetDeviceConfigurationForOpMode();
        }



        //Gamepad2


        // intake power
        float intakePower = -gamepad2.left_stick_y;
        robot.intake.setPower(intakePower);




        telemetry.addData("Encoder", robot.arm.getCurrentPosition());
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
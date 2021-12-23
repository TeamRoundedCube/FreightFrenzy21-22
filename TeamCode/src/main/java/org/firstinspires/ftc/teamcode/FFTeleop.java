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
import com.qualcomm.robotcore.hardware.DcMotor;

//Created by Kyran 10/16/2021 @ 3:05pm
//Purpose: Marc will use to test the new competition robot


@TeleOp(name = "FFTeleop")
public class FFTeleop extends OpMode {

    FFHardwareFullBot robot = new FFHardwareFullBot();
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
        telemetry.addData("leftx:", Math.abs(gamepad1.left_stick_x));
        telemetry.addData("lefty:", Math.abs(gamepad1.left_stick_y));
        telemetry.update();
        // */
        // Diagonal
        boolean diagonalY = (Math.abs(gamepad1.left_stick_y) > 0.3 && Math.abs(gamepad1.left_stick_y) < 0.8);
        boolean diagonalX = (Math.abs(gamepad1.left_stick_x) > 0.3 && Math.abs(gamepad1.left_stick_x) < 0.8);
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

        float maxCarousel = (float) 0.75;

        robot.spincarousel.setPower((gamepad1.right_trigger)*maxCarousel);
            //robot.spincarousel.setPower(1);
        robot.spincarousel.setPower(-(gamepad1.left_trigger)*maxCarousel);
            //robot.spincarousel.setPower(-1);



        //Gamepad2


        // intake power
        float intakePower = -gamepad2.left_stick_y;
        //float intakePower = -1; - changed by Savita for Marc testing on 12-12-21
        robot.intake.setPower(intakePower);



        // Arm Position Values

        int downPosition = 0;
        int drivingPosition = 350;
        int levelOne = 3000;
        int levelTwo = 2600;
        int levelThree = 2000;
        double armSpeed = 0.75;

        //Arm
        if (gamepad2.dpad_down) { //Intake position
            moveArm(armSpeed, downPosition);
            robot.basket.setPosition(0.35);
        } else if (gamepad2.dpad_right) { // Level 1
            moveArm(armSpeed, levelOne);

        } else if (gamepad2.dpad_left) { //Level 2
            moveArm(armSpeed, levelTwo);

        } else if (gamepad2.dpad_up) { //Level 3
            //sleep(1000);
            moveArm(armSpeed, levelThree);

        } else if (gamepad2.right_bumper) { //Driving Position
            //testArm();
            moveArm(armSpeed, drivingPosition);

        }


        if(gamepad2.y) {
            robot.basket.setPosition(0.35);
            if(robot.arm.getCurrentPosition() != drivingPosition) {
                moveArm(armSpeed, drivingPosition);
            }
            //sleep(1000);
        /*} else if(gamepad2.b) {
            robot.basket.setPosition(0.35);
            //sleep(1000);*/
        } else if (gamepad2.x) {
            robot.basket.setPosition(0);
            sleep(1000);
            if(robot.arm.getCurrentPosition() != downPosition) {
                moveArm(armSpeed, downPosition);
                robot.basket.setPosition(0.35);
                sleep(300);
            }
        }
// 0.95 - 0.45
        if(gamepad2.b) {
            robot.element.setPosition(0.95);
        }
        if(gamepad2.a) {
            robot.element.setPosition(0.45);
        }
/*
        //X button Override

        if (gamepad2.x) {
            robot.arm.setPower(0);
            robot.basket.setPosition(0.35);
            sleep(1000);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.claw.resetDeviceConfigurationForOpMode();
        }
*/
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
        telemetry.addData("LeftTrigger", gamepad1.left_trigger);
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

    public void moveArm(double speed, int position) {
        robot.arm.setPower(speed);
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.arm.isBusy()) {
            sleep(1);
        }
        robot.arm.setPower(0);
    }

    public void testArm() {
        robot.arm.setTargetPosition(3000);
        telemetry.addData("position", "position");
        telemetry.update();
        robot.arm.setPower(0.5);
        telemetry.addData("power", "power");
        telemetry.update();
        // Switch to RUN_TO_POSITION mode
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("run", "position");
        telemetry.update();

        // Start the motor moving by setting the max velocity to 200 ticks per second


        // While the Op Mode is running, show the motor's status via telemetry
        /*while (robot.arm.isBusy()) {
            telemetry.addData("is", "busy");
            telemetry.update();
           sleep(1);
        }*/
        telemetry.addData("not", "busy");
        telemetry.update();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
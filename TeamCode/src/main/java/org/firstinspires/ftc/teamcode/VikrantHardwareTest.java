/* Copyright (c) 2017 FIRST. All rights reserved.
 * This is a test program to see if the hardware is working
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="VikrantHardwareTest", group="Util")
@Disabled
public class VikrantHardwareTest extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    DcMotor back_left, back_right, front_left, front_right;

    double power = 1.0;


    @Override
    public void runOpMode() {

        /*back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();
        */
        front_left  = hardwareMap.get(DcMotorEx.class, "front_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_left    = hardwareMap.get(DcMotorEx.class, "back_left");
        back_right    = hardwareMap.get(DcMotorEx.class, "back_right");

        front_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        front_right.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        back_left.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        back_right .setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark moto
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            back_left.setPower(power);
            back_right.setPower(power);
            front_left.setPower(power);
            front_right.setPower(power);
        }
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }
}

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "AutoTest", group = "Concept")
//@Disabled
public class AutoTest extends LinearOpMode {


    //Drive Motors
    DcMotor back_left;
    DcMotor back_right;
    DcMotor front_left;
    DcMotor front_right;

    //Carousel Spinning Motor- Vikrant
    //DcMotor carousel_spinner;

    //Arm to drop block on shipping hub
    //DcMotor arm;
    //Servo claw;
    private ElapsedTime runtime = new ElapsedTime();
    HardwareFullBot robot = new HardwareFullBot();

    //Variable for levels on shipping hub- Vikrant

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        //HardwareMap for Drive Motors and arm- Vikrant
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");

        //arm = hardwareMap.get(DcMotor.class, "arm");

        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //Temporary Remove after testing (3 lines)
               // StrafeRightforTime(1, 5.5, 0);
                DriveforTime(0.5, 1, 0);

                //Move back 2 inches- Vikrant
                //DriveforTime(-1, 0.25, 0);

                //Turn right to park in warehouse- Vikrant
                //turnRight(1, 2.5, 0);

                //Drive to warehouse and park inside warehouse- Vikrant
                //DriveforTime(1, 5.5, 0);

            }
        }
    }

        //Function to make robot turn right

        public void turnRight ( double speed,
        double seconds,
        double timeoutS){

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                runtime.reset();
                back_right.setPower(0);
                back_left.setPower(speed);
                front_right.setPower(0);
                front_left.setPower(speed);
                seconds = seconds * 1000;
                double i = 0;
                while (opModeIsActive() && i < seconds &&
                        (runtime.seconds() < timeoutS)) {
                    sleep(1);
                    i++;
                }

                // Stop all motion;
                front_right.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);

            }
        }


        //Drive forward for a certain number of seconds
        public void DriveforTime ( double speed,
        double seconds,
        double timeoutS){

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                runtime.reset();
                back_right.setPower(speed);
                back_left.setPower(speed);
                front_right.setPower(speed);
                front_left.setPower(speed);
                seconds = seconds * 1000;
                double i = 0;
                while (opModeIsActive() && i < seconds &&
                        (runtime.seconds() < timeoutS)) {
                    sleep(1);
                    i++;
                }

                // Stop all motion;
                front_right.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);

            }
        }


        public void StrafeLeftforTime ( double speed,
        double seconds,
        double timeoutS){

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                runtime.reset();
                back_right.setPower(speed);
                back_left.setPower(-speed);
                front_right.setPower(speed);
                front_left.setPower(-speed);
                seconds = seconds * 1000;
                double i = 0;
                while (opModeIsActive() && i < seconds &&
                        (runtime.seconds() < timeoutS)) {
                    sleep(1);
                    i++;
                }

                // Stop all motion;
                front_right.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);

            }
        }


        public void StrafeRightforTime ( double speed,
        double seconds,
        double timeoutS){

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                runtime.reset();
                back_right.setPower(-speed);
                back_left.setPower(speed);
                front_right.setPower(-speed);
                front_left.setPower(speed);
                seconds = seconds * 1000;
                double i = 0;
                while (opModeIsActive() && i < seconds &&
                        (runtime.seconds() < timeoutS)) {
                    sleep(1);
                    i++;
                }

                // Stop all motion;
                front_right.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);

            }
        }


    }


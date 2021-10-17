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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "FreightFrenzyAuto_WithoutVuforia", group = "Concept")
//@Disabled
public class FreightFrenzyAuto_WithoutVuforia extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareFullBot robot = new HardwareFullBot();
    @Override
    public void runOpMode()
    {
        //HardwareMap for Drive Motors and arm- Vikrant
        robot.init(hardwareMap);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive())
        {
 //           while (opModeIsActive())
 //           {
                //Step 1: scan the duck, set level
                //Step 2: strafe left towards carousel
                StrafeLeftforTime(1, 4);
                sleep(1000);
                //Step 2.5: SpinCarousel(1,1)
                sleep(1000);
                //Step 3: Move front for 2 seconds - Prabhav
                DriveforTime(1, 4);
                sleep(1000);
                //Step 4: Move back for 2 seconds - Vikrant
                DriveforTime(-1, 4);
                sleep(1000);
                //Step 5: Turn right
                turnRight(1, 10.5);
                sleep(1000);
                //Step 6: Drive to warehouse and park inside warehouse- Vikrant
                DriveforTime(1, 20);
                sleep(1000);
 //           }
        }
    }
    //Vikrant
  public void turnRight(double speed,
                          double seconds) {
        //For this method it takes 5 seconds to turn 90 degrees-Prabhav-10/17/2021
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds*1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds ) {

                robot.front_left.setPower(speed);
                robot.front_right.setPower(-speed);
                robot.back_left.setPower(speed);
                robot.back_right.setPower(-speed);

                i++;
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }
    public void turnLeft(double speed,
                          double seconds) {
        //For this method it takes 5 seconds to turn 90 degrees-Prabhav-10/17/2021
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds*1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds ) {

                robot.front_left.setPower(-speed);
                robot.front_right.setPower(speed);
                robot.back_left.setPower(-speed);
                robot.back_right.setPower(speed);

                i++;
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }


    //Drive forward for a certain number of seconds
    public void DriveforTime(double speed,
                             double seconds) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds*1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds ) {

                robot.front_right.setPower(speed);
                robot.back_right.setPower(speed);
                robot.front_left.setPower(speed);
                robot.back_left.setPower(speed);

                i++;
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }


    public void StrafeLeftforTime(double speed,
                                  double seconds) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds*1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds ) {

                robot.front_right.setPower(speed);
                robot.back_right.setPower(-speed);
                robot.front_left.setPower(-speed);
                robot.back_left.setPower(speed);

                i++;
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }


    public void StrafeRightforTime(double speed,
                                   double seconds) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds*1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds ) {

                robot.front_right.setPower(-speed);
                robot.back_right.setPower(speed);
                robot.front_left.setPower(speed);
                robot.back_left.setPower(-speed);

                i++;
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }
    /*
    public void SpinCarousel(double speed,
                             double seconds) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds*1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds ) {

                spincarousel.setPower(speed)

                i++;
            }

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }
    */

    //Function to drop block on correct level- Vikrant
    /*
    public void dropBlockLevel1(){

        if(opModeIsActive()) {

            robot.arm.setPower(0.5);
            moveArmDown(-70);
            clawOpen();
            clawClose();
            moveArmUp(0);

        }


    }

    public void dropBlockLevel2(){

        if(opModeIsActive()) {

            robot.arm.setPower(0.5);
            moveArmDown(-50);
            clawOpen();
            clawClose();
            moveArmUp(0);

        }
    }

    public void dropBlockLevel3(){

        if(opModeIsActive()) {

            robot.arm.setPower(0.5);
            moveArmDown(-30);
            clawOpen();
            clawClose();
            moveArmUp(0);
        }
    }

    //arm and claw control functions
    public void clawOpen() {
        clawControl(0);
    }
    public void clawClose() {
        clawControl(1);
    }
    public void clawControl(double position) {

        claw.setPosition(position);
        sleep(2000);

    }
    //Function that makes the arm move downwards
    public void moveArmDown(double angle) {

        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.arm.setPower(0.6);
        sleep(600);

        arm.setPower(0);




    }

    //Function that moves arm up
    public void moveArmUp(double angle) {
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.arm.setPower(-0.8);
        sleep(500);

        robot.arm.setPower(0);



    }
*/
}
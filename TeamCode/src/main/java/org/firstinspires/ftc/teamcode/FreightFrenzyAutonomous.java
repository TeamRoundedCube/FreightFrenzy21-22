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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
@Autonomous(name = "FreightFrenzyAutonomous", group = "Concept")
@Disabled
public class FreightFrenzyAutonomous extends LinearOpMode {
  /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */

    //Drive Motors- Vikrant
    /*
    DcMotor back_left;
    DcMotor back_right;
    DcMotor front_left;
    DcMotor front_right;
    */
    //Carousel Spinning Motor- Vikrant
    //DcMotor carousel_spinner;

    //Arm to drop block on shipping hub
    //DcMotor arm;
    //Servo claw;

    //Variable for levels on shipping hub- Vikrant
    //level too generic
    int level = 0;

    private ElapsedTime runtime = new ElapsedTime();

    KyranHardwareFullBot robot = new KyranHardwareFullBot();
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";//-where is this used in code?
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AenEeff/////AAABmVfM1TTQPUGRpC7PQZbNw25BExLdZ2MNuZfotDeDWeTSu/9GUq44dVnQHBGowhhEMplBNQLlvJ5Ai05PWcHybchVql4+VlfHGFu137Sc9dZgUEWSLYGmLcs4OG0HaX6qWsz9O5A0v/JbTJAMQHmF+DXSP0p1nIhiALIIU9jw7LnG+ik4k+xWAoFRdXVEzJMm8yVCjwZlowJjQd0hKRHXbJQG26T/rAEl4LwB9unLfnV6SsyEhUexEKPynizjgWMfqfbSlXUvBNqwURNXhBHmuGscNsbycdENESR89r0V1bZ0C/lOMs56VNfoi1G6+0u4JV9MF3gqaQx8xbzjE7B87ligZz87k5lpCnbbA/AfqZ6Q";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //HardwareMap for Drive Motors and arm- Vikrant
        /*
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");

        //arm = hardwareMap.get(DcMotor.class, "arm");

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        */
        initVuforia();
        initTfod();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            //Step 1- Drive to carousel
           // DriveforTime(.55, 4.5);
           // sleep(1000);
            //Step 2- Spin carousel
           // SpinCarousel(-.6,53);
           // sleep(1000);
            //Step 3- Strafe right to move away from carousel
           // StrafeRightforTime(.5, .5);
            //sleep(1000);
            //Step 4- Turn right to line up with wall
           // turnRight(1, 2.5);
            //sleep(100);
            //Step 5- Strafe right to line up with the bar codes
           // StrafeRightforTime(.5, 1);
            //sleep(1000);

            //Step 6- detect the position of the duck/teamshipping element
            //while (opModeIsActive()) {

            DriveforTime(1, .35);
            sleep(1000);
            while (level == 0) {
                 if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("Entire Image Width", recognition.getImageWidth());
                            telemetry.addData("Entire Image Height", recognition.getImageHeight());
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.addData("Image Height", recognition.getHeight());
                            telemetry.addData("Image Width", recognition.getWidth());
                            i++;
                        }
                    }
                     telemetry.update();
                      //telemetry.clearAll();
                      //if Statement to determine level for shipping hub and display level on driver station- Vikrant
                      if (updatedRecognitions != null) {
                          for (Recognition recognition : updatedRecognitions) {
                              double xCoordinate = recognition.getLeft();
                              telemetry.addData("Item", recognition.getLabel());
                              telemetry.addData("Duck-X", xCoordinate);
                              telemetry.update();
                              if (recognition.getLabel() == "Duck") {
                                  telemetry.addData("X-coordinate", xCoordinate);
                                  if (xCoordinate < 250) {
                                      level = 1;
                                      telemetry.addData("Level", level);
                                      telemetry.update();
                                      break;
                                  } else if (xCoordinate > 250) {
                                      level = 2;
                                      telemetry.addData("Level", level);
                                      telemetry.update();
                                      break;
                                  } else {
                                      level = 3;
                                      telemetry.addData("Level", level);
                                      telemetry.update();
                                      break;
                                  }
                              }
                              else if (recognition.getLabel() == "Marker"){
                                  level = 3;
                                  telemetry.addData("Level", level);
                                  telemetry.update();
                                  break;
                              }
                          }
                      }
                      // If for some reason, the robot does not go forward enough and it does not recognize any items
                      // and level is not set yet
                      // and if it is past seconds
                      // then default to level 3
                     else if (updatedRecognitions == null && level == 0 && runtime.seconds() > 5){
                          level = 3;
                          telemetry.addData("Level", level);
                          telemetry.update();
                          break;
                      }
                 }
            }
            telemetry.update();

            //Step 7- Strafe right to line up with the shipping hub
            StrafeRightforTime(.75, 5);
            //sleep(1000);
            //Step 8- adjust arm based on level
            //Define levels
            int levelOne = 2950;
            int levelTwo = 2455;
            int levelThree = 1950;
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (level == 1) {
                robot.arm.setPower(-0.35);
                robot.arm.setTargetPosition(levelOne);// Level 1
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);
                DriveforTime(.5, 1.05);
                //sleep(1000);
                robot.basket.setPosition(0.3);
                sleep(1000);
                robot.basket.setPosition(0.18);
                sleep(1000);
                DriveforTime(-.5, 1.05);
                //sleep(1000);
                robot.arm.setTargetPosition(500);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);
            } else if (level == 2) {
                robot.arm.setPower(-0.35);
                robot.arm.setTargetPosition(levelTwo);//Level 2
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);
                DriveforTime(.5, 1.25);
                //sleep(1000);
                robot.basket.setPosition(0.3);
                sleep(1000);
                robot.basket.setPosition(0.18);
                sleep(1000);
                DriveforTime(-.5, 1.25);
                //sleep(1000);
                robot.arm.setTargetPosition(500);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);
            } else if (level == 3) {
                robot.arm.setPower(-0.35);
                robot.arm.setTargetPosition(levelThree);//Level 3
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);
                DriveforTime(.5, 1.45);
                //sleep(1000);
                robot.basket.setPosition(0.3);
                sleep(1000);
                robot.basket.setPosition(0.18);
                sleep(1000);
                DriveforTime(-.5, 1.45);
                //sleep(1000);
                robot.arm.setTargetPosition(500);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);
            }
            // strafeleft to original position
            StrafeLeftforTime(.5, 7);
            sleep(1000);
            StrafeLeftforTime(.25, 1);
            sleep(1000);
            //Drive back to original starting position
            //DriveforTime(.5, 1);
            //Turn Left
            turnLeft(.25,6);
            sleep(1000);
            //Drive forward to reach the carousel
            //DriveforTime(.55, 4.5);
            //sleep(1000);
            //Step 2.5: Spin Carousel with max power for 1 sec
            //SpinCarousel(-.6,53);
            //sleep(1000);
            //Step 3: Strafe right away from carousel
            //StrafeRightforTime(1, 2);
            //sleep(1000);
            //Step 4: Drive out of storage unit
            //DriveforTime(-1, 4);
            //sleep(1000);
            //Step 4: Turn left to line up with wall
            //turnLeft(1, .3);
            //sleep(1000);
            //Step 5: Strafe left to line up with wall
            //StrafeLeftforTime(1, 9);
            //sleep(1000);
            //Step 5: Drive Backwards To Warehouse
            //DriveforTime(-1,13.5);
                }
            }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    //Function to make robot turn right

        public void turnRight(double speed,
                              double seconds) {

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                runtime.reset();
                double milliseconds = seconds*1000;
                double i = runtime.milliseconds();
                while (opModeIsActive() && i < milliseconds ) {

                    robot.front_right.setPower(0);
                    robot.back_right.setPower(0);
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

    public void SpinCarousel(double speed,
                             double seconds) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds*1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds ) {

                robot.spincarousel.setPower(speed);


                i++;
            }

            // Stop all motion;
            robot.spincarousel.setPower(0);
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }

    public void driveInches(double speed, double inches) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int FRtarget = (int) (-50*inches);
            int FLtarget = (int) (-40*inches);
            int BRtarget = (int) (-50*inches);
            int BLtarget = (int) (-30*inches);

            robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            runtime.reset();

            robot.front_right.setTargetPosition(FRtarget);
            robot.front_left.setTargetPosition(FLtarget);
            robot.back_right.setTargetPosition(BRtarget);
            robot.back_left.setTargetPosition(BLtarget);

            // set motors to run to target encoder position and stop with brakes on.
            robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.front_right.setPower(-speed);
            robot.back_right.setPower(-speed);
            robot.front_left.setPower(-speed);
            robot.back_left.setPower(-speed);
/*
            while (opModeIsActive() && robot.front_left.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            {
               // telemetry.addData("encoder-fwd-left", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
               // telemetry.addData("encoder-fwd-right", rightMotor.getCurrentPosition() + "  busy=" + rightMotor.isBusy());
                telemetry.update();
                idle();
            }
*/
/*
            while (opModeIsActive() &&
                    (FRtarget < robot.front_right.getCurrentPosition() ||
                    FLtarget < robot.front_left.getCurrentPosition() ||
                    BRtarget < robot.back_right.getCurrentPosition() ||
                    BLtarget < robot.back_left.getCurrentPosition())) {

                robot.front_right.setPower(-speed);
                robot.back_right.setPower(-speed);
                robot.front_left.setPower(-speed);
                robot.back_left.setPower(-speed);

            }*/

            // Stop all motion;
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }


    //Function to drop block on correct level- Vikrant
    /*
    public void dropBlockLevel1(){

        if(opModeIsActive()) {

            arm.setPower(0.5);
            moveArmDown(-70);
            clawOpen();
            clawClose();
            moveArmUp(0);

        }


    }

    public void dropBlockLevel2(){

        if(opModeIsActive()) {

            arm.setPower(0.5);
            moveArmDown(-50);
            clawOpen();
            clawClose();
            moveArmUp(0);

        }
    }

    public void dropBlockLevel3(){

        if(opModeIsActive()) {

            arm.setPower(0.5);
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

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0.6);
        sleep(600);

        arm.setPower(0);




    }

    //Function that moves arm up
    public void moveArmUp(double angle) {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(-0.8);
        sleep(500);

        arm.setPower(0);



    }
*/
}

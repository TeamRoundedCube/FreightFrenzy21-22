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
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
@Autonomous(name = "FFAuto", group = "Concept")
//@Disabled
public class FFAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //HardwareFullBot robot = new HardwareFullBot();
    FFHardwareFullBot robot = new FFHardwareFullBot();
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";//-where is this used in code?
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    static final double COUNTS_PER_MOTOR_REV = 400;    // 537 (Original)
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.9;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double LEFT_FRONT_COEFF = 1.0;
    static final double LEFT_BACK_COEFF = 1.0;
    static final double RIGHT_FRONT_COEFF = 1.0;
    static final double RIGHT_BACK_COEFF = 1.0;

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
    //Variable for levels on shipping hub- Vikrant
    //level too generic
    int level = 0;

    @Override
    public void runOpMode() {
        int downPosition = 0;
        int drivingPosition = 350;
        int levelOne = 3300;
        int levelTwo = 2900;
        int levelThree = 2300;
        double armSpeed = 0.25;

        //HardwareMap for Drive Motors and arm- Vikrant
        robot.init(hardwareMap);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        initVuforia();
        initTfod();
        while (opModeIsActive() == false) {
            detectObject();
        }
        waitForStart();

        if (opModeIsActive()) {
            telemetry.clear();
            telemetry.addData("Level at start:", level);
            telemetry.update();
            //turnLeft(.25, 2);
            stop();
            StrafeRightforTime(.5, .22);
            //sleep(1000);
            driveForward(.1, 13);
            sleep(1000);
            //Step 2.5: Spin Carousel with max power for 1 sec
            SpinCarousel(-.5, 2);
            //sleep(1000);
            driveReverse(.25, 61);
            //sleep(1000);
            turnRight(.5, .570);
            //sleep(1000);
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            moveArm(armSpeed, levelThree);
            sleep(1000);
            driveForward(.25, 18);
            sleep(500);
            robot.basket.setPosition(1);
            sleep(500);
            robot.basket.setPosition(.6);
            //sleep(1000);
            moveArm(.75, drivingPosition);
            driveReverse(.25, 18);
            //sleep(1000);
            turnLeft(.5, .58);
            //sleep(1000);
            StrafeLeftforTime(.5, .20);
            //Step 4: Drive to Warehouse
            driveReverse(.75, 65);
            moveArm(.5, downPosition);
            //           }
        }
    }

    public void turnRight(double speed,
                          double seconds) {
        //For this method it takes 5 seconds to turn 90 degrees-Prabhav-10/17/2021
        // Ensure that the opmode is still active
        robot.front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            double milliseconds = seconds * 1000;
            double i = 0;
            while (opModeIsActive() && i < milliseconds) {

                robot.front_left.setPower(speed);
                robot.front_right.setPower(-speed);
                robot.back_left.setPower(speed);
                robot.back_right.setPower(-speed);

                sleep(1);
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
        robot.front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            double milliseconds = seconds * 1000;
            double i = 0;
            while (opModeIsActive() && i < milliseconds) {

                robot.front_left.setPower(-speed);
                robot.front_right.setPower(speed);
                robot.back_left.setPower(-speed);
                robot.back_right.setPower(speed);

                sleep(1);
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
        robot.front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

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
        robot.front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double milliseconds = seconds * 1000;
            double i = 0;
            while (opModeIsActive() && i < milliseconds) {

                robot.front_right.setPower(speed);
                robot.back_right.setPower(-speed);
                robot.front_left.setPower(-speed);
                robot.back_left.setPower(speed);

                sleep(1);
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
        robot.front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double milliseconds = seconds * 1000;
            double i = 0;
            while (opModeIsActive() && i < milliseconds) {

                robot.front_right.setPower(-speed);
                robot.back_right.setPower(speed);
                robot.front_left.setPower(speed);
                robot.back_left.setPower(-speed);

                sleep(1);
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

            double milliseconds = seconds * 1000;
            double i = 0;
            while (opModeIsActive() && i < milliseconds) {

                robot.spincarousel.setPower(speed);

                sleep(1);
                i++;
            }

            // Stop all motion;
            robot.spincarousel.setPower(0);

        }
    }


    public void driveReverse(double speed, double distance) {
        encoderDriveReverse(speed, distance, distance, 30);

    }

    public void driveForward(double speed, double distance) {
        encoderDrive(speed, distance, distance, 30);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newbackLeftTarget = robot.back_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.back_right.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newfrontLeftTarget = robot.front_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.front_right.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.back_left.setTargetPosition(newbackLeftTarget);
            robot.back_right.setTargetPosition(newbackRightTarget);
            robot.front_right.setTargetPosition(newfrontRightTarget);
            robot.front_left.setTargetPosition(newfrontLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.

            runtime.reset();
            robot.back_right.setPower(Math.abs(speed));
            robot.back_left.setPower(Math.abs(speed));
            robot.front_left.setPower(Math.abs(speed));
            robot.front_right.setPower(Math.abs(speed));

/*
            robot.back_right.setPower(Math.abs(speed));
            robot.back_left.setPower(Math.abs(speed));
            robot.front_left.setPower(Math.abs(speed));
            robot.front_right.setPower(Math.abs(speed));
*/
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() && (robot.back_left.getCurrentPosition() < newbackLeftTarget) && (robot.back_right.getCurrentPosition() < newbackRightTarget) &&
                    (runtime.seconds() < timeoutS))

         /*                   while (opModeIsActive()
                                    && robot.back_left.isBusy()
                                    && robot.back_right.isBusy()
                                    && robot.front_right.isBusy()
                                    && robot.front_left.isBusy()
                                    &&  runtime.seconds() < timeoutS)*/ {
                //Provides current position and updates it every time it changes
                telemetry.addData("Curr Velocity at time ", "backleft(%.2f), " +
                                "backright (%.2f)",
                        robot.back_left.getVelocity(),
                        robot.back_right.getVelocity());
                //sleep(250);
                telemetry.update();
                idle();
            }
/*
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    &&
                    (robot.front_right.isBusy())
            ) {


                telemetry.addData("target", "Running to %7d :%7d   :%7d  :%7d", newbackLeftTarget
                        , newbackRightTarget
                        , newfrontLeftTarget
                        , newfrontRightTarget);
                telemetry.addData("CurrentPositon", "Running at %7d :%7d  :%7d  :%7d",
                        robot.front_left.getCurrentPosition(),
                        robot.front_right.getCurrentPosition(),
                        robot.back_left.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();

            }
*/
            // Stop all motion;
            robot.front_left.setPower(0);
            robot.back_right.setPower(0);
            robot.front_right.setPower(0);
            robot.back_left.setPower(0);
        }
    }

    public void encoderDriveReverse(double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS) {
        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        telemetry.addData("DriveReverse", "Running at %7d :%7d  :%7d  :%7d",
                robot.front_left.getCurrentPosition(),
                robot.front_right.getCurrentPosition(),
                robot.back_left.getCurrentPosition(),
                robot.back_right.getCurrentPosition());
        telemetry.update();
        // Determine new target position, and pass to motor controller
        newbackLeftTarget = robot.back_left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
        newbackRightTarget = robot.back_right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
        newfrontLeftTarget = robot.front_left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
        newfrontRightTarget = robot.front_right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);

        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.back_left.setTargetPosition(newbackLeftTarget);
        robot.back_right.setTargetPosition(newbackRightTarget);
        robot.front_right.setTargetPosition(newfrontRightTarget);
        robot.front_left.setTargetPosition(newfrontLeftTarget);

        // Turn On RUN_TO_POSITION
        robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        while (opModeIsActive()
                && (robot.front_left.getCurrentPosition() > newfrontLeftTarget)
                && (robot.front_right.getCurrentPosition() > newfrontRightTarget)
                // && (robot.back_right.getCurrentPosition() > newbackRightTarget)
                // && (robot.back_left.getCurrentPosition() > newbackLeftTarget)
                && (runtime.seconds() < timeoutS)) {

            robot.back_right.setPower(-speed * RIGHT_BACK_COEFF);
            robot.front_right.setPower(-speed * RIGHT_FRONT_COEFF);
            robot.back_left.setPower(-speed * LEFT_BACK_COEFF);
            robot.front_left.setPower(-speed * LEFT_FRONT_COEFF);

        }


/*
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS)
                &&
                (robot.front_right.isBusy())
                &&
                (robot.front_left.isBusy())
                &&
                (robot.back_right.isBusy())
                &&
                (robot.back_left.isBusy())
        ) {


            telemetry.addData("target", "Running to %7d :%7d   :%7d  :%7d", newbackLeftTarget
                    , newbackRightTarget
                    , newfrontLeftTarget
                    , newfrontRightTarget);
            telemetry.addData("CurrentPositon", "Running at %7d :%7d  :%7d  :%7d",
                    robot.front_left.getCurrentPosition(),
                    robot.front_right.getCurrentPosition(),
                    robot.back_left.getCurrentPosition(),
                    robot.back_right.getCurrentPosition());
            telemetry.update();

        }*/
        robot.front_right.setPower(0);    // Stop all motion;
        robot.back_right.setPower(0);
        robot.front_left.setPower(0);
        robot.back_left.setPower(0);


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

    public void moveArm(double speed, int position) {
        robot.arm.setPower(speed);
        robot.arm.setTargetPosition(position);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.arm.isBusy()) {
            sleep(1);
        }
        robot.arm.setPower(0);
    }

    public void detectObject() {

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
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        //Scan the duck, set level
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
                    telemetry.addData("i:",i);
                    i++;
                    double xCoordinate = recognition.getLeft();
                    if (recognition.getLabel() == "Duck") {
                        if (xCoordinate <= 250) {
                            level = 1;
                            telemetry.addData("Level xCoordinate<250", level);
                            telemetry.update();
                            break;
                        } else if (xCoordinate > 250) {
                            level = 2;
                            telemetry.addData("Level xCoordinate>250", level);
                            telemetry.update();
                            break;
                        }
                    }
                    else {level = 3;
                        telemetry.addData("Else part of Level recognition.getLabel()==Duck", level);
                        telemetry.update();
                        break;
                    }
                }

                //telemetry.clearAll();
                //if Statement to determine level for shipping hub and display level on driver station- Vikrant
                /*if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        double xCoordinate = recognition.getLeft();
                        if (recognition.getLabel() == "Duck") {
                            if (xCoordinate < 100) {
                                level = 1;
                                telemetry.addData("Level", level);
                                telemetry.update();
                            } else if (xCoordinate > 250 && xCoordinate < 400) {
                                level = 2;
                                telemetry.addData("Level", level);
                                telemetry.update();
                            }
                        }
                    }
                }*/

            }
            else if (level !=1 && level !=2){
                level = 3;
                telemetry.addData("Else part of updated recognitions != null:", level);
                telemetry.update();
            }

            }
        }
    }

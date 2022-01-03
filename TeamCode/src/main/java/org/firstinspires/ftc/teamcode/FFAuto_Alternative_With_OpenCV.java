package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "FFAuto_Alternative_With_OpenCV", group = "Concept")
public class FFAuto_Alternative_With_OpenCV extends LinearOpMode {
    //Webcam variable
    OpenCvWebcam webcam;
    GyroSensor gyro = null;

    FFHardwareFullBot robot = new FFHardwareFullBot();


    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 400;    // 537 (Original)
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.9;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double LEFT_FRONT_COEFF = 1.0;
    static final double LEFT_BACK_COEFF = 1.0;
    static final double RIGHT_FRONT_COEFF = 1.0;
    static final double RIGHT_BACK_COEFF = 1.0;



    @Override
    public void runOpMode() {
        int downPosition = 0;
        int drivingPosition = 350;
        int levelOne = 3300;
        int levelTwo = 2850;
        int levelThree = 2300;
        double armSpeed = 1;
        //int rawX = gyro.rawX();
        //int rawY = gyro.rawY();

        robot.init(hardwareMap);

        int level = 0;

        //Instance of OpenCV class
        FFOpenCVPipelineClass opencv = new FFOpenCVPipelineClass();
        //FFOpenCV opencv;

        //Initializing Camera
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));

        //The OpenCV class is determining what is being shown on the camera. It is the pipeline
        //webcam.setPipeline(opencv);

        //webcam.setMillisecondsPermissionTimeout(2500);

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        webcam.setPipeline(new FFOpenCVPipelineClass());
        webcam.setMillisecondsPermissionTimeout(2500);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {


            }
        });

        while (opModeIsActive() == false) {

            if (opencv.getLocation() == FFOpenCVPipelineClass.Location.LEFT) {
                level = 1;
                telemetry.addData("Level", 1);
                telemetry.update();
                sleep(3000);

            } else if (opencv.getLocation() == FFOpenCVPipelineClass.Location.MIDDLE) {
                level = 2;
                telemetry.addData("Level", 2);
                telemetry.update();
                sleep(3000);

            } else if (opencv.getLocation() == FFOpenCVPipelineClass.Location.RIGHT) {
                level = 3;
                telemetry.addData("Level", 3);
                telemetry.update();
                sleep(3000);

            } else if (opencv.getLocation() == FFOpenCVPipelineClass.Location.NOTHING) {
                level = 0;
                telemetry.addLine("Nothing Detected");
                telemetry.update();
                sleep(3000);

            }


            /*//telemetry.addData("Mode", "starting gyro calibration...please wait");
            //telemetry.update();
            //telemetry.addData("X value",rawX);
            //telemetry.addData("Y Value",rawY);
            telemetry.addLine("Waiting for start");
            telemetry.update();
            robot.gyro.calibrate();
            robot.gyro.resetZAxisIntegrator();
            // make sure the gyro is calibrated before continuing
            while (isStopRequested() == false && robot.gyro.isCalibrating()) {
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "gyro calibrated...waiting for start");
            telemetry.update();
            telemetry.addData("Status: ", robot.gyro.status());*/
        }
        // wait for start button.

        waitForStart();

        //Uses getLocation function from OpenCV class to find and display Level
        if (opModeIsActive()) {
            //robot.gyro.calibrate();
            //rotate(90,0.5);
            //rotate(-90,0.5);
            telemetry.clear();
            telemetry.addData("Level at start:", level);
            telemetry.update();
            driveForward(.5, 3);
            //turnLeft(.25, 1.2);
            StrafeLeftforTime(0.5,0.58);
            //StrafeRightforTime(0.25, .5);
            //sleep(1000);
            //driveForward(.4, 23);
            //driveForward(.1, 4.5);
            //driveForward(.1, 4.25);
            //sleep(1000);
            //Step 2.5: Spin Carousel with max power for 1 sec
            //SpinCarousel(-.5, 2);
            //sleep(1000);
            //driveReverse(.5, 57);
            //sleep(500);
            //turnRight(.25, 1.2);
            //sleep(1000);
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            if (level == 1) {
                moveArm(armSpeed, levelOne);
                sleep(500);
                driveForward(.5, 6);

            }
            else if (level == 2) {

                moveArm(armSpeed, levelTwo);
                sleep(500);
                driveForward(.5, 9);


            }
            else if (level == 3) {

                moveArm(armSpeed, levelThree);
                sleep(500);
                driveForward(.5, 15);


            }
            else if (level == 0) {

                telemetry.addLine("Nothing Detected");
                telemetry.update();
                sleep(3000);

            }

            robot.basket.setPosition(1);
            sleep(1000);

            if (level == 1) {
                driveReverse(.25, 6);
            }
            else if (level == 2) {
                driveReverse(.25, 9);
            }
            else if (level == 3) {
                driveReverse(.25, 15);
            }
            else if (level == 0) {

                telemetry.addLine("Nothing Detected");
                telemetry.update();
                sleep(3000);

            }

            robot.basket.setPosition(.35);
            sleep(500);
            moveArm(1, downPosition);
            //sleep(1000);
            turnLeft(.5, .52);
            //sleep(1000);
            StrafeLeftforTime(.5, .5);
            //Step 4: Drive to Warehouse
            driveReverse(.5, 64);
            //moveArm(.5, downPosition);
            //           }
            // Start intake, Drive forward to shipping hub, straiph left a little,
            // turn and drop ball or cube inside level.
            intakecollect(1, 1.5);
            driveForward(.5,77);
            StrafeRightforTime(.25, .35);
            turnRight(.25, 1.2);
            moveArm(armSpeed, levelThree);
            driveForward(.5, 15);
            robot.basket.setPosition(1);
            sleep(500);
            robot.basket.setPosition(.35);
            sleep(500);
            moveArm(armSpeed,downPosition);
            driveReverse(1, 15);
            turnLeft(.5, .52);
            StrafeLeftforTime(.5, .5);
            driveReverse(.75, 65);
            StrafeRightforTime(.5, .7);



        }


    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, heading, gain = .10;

        heading = robot.gyro.getHeading();

        if (heading == 0)
            correction = 0;             // no adjustment.
        else if (heading > 180)
            correction = 360 - heading; // adjust left.
        else
            correction = -heading;      // adjust right.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 350 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        double  leftPower;
        double rightPower;
        int     targetAngle = degrees;

        robot.front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // reset gyro to zero.
        //robot.gyro.resetZAxisIntegrator();

        // Gyro returns 0->359 when rotating counter clockwise (left) and 359->0 when rotating
        // clockwise (right).
        if (degrees < 0)
        {   // turn right.
           // while (degrees < 0){
           //     targetAngle++;
           // }
            leftPower = power;
            rightPower = -power;
            robot.front_left.setPower(leftPower);
            robot.back_left.setPower(leftPower);
            robot.front_right.setPower(rightPower);
            robot.back_right.setPower(rightPower);

            //degrees -= targetAngle;
            // degrees is - for right turn.
        }
        else if (degrees > 0)
        {   // turn left.
            //while (degrees > 0){
            //    targetAngle++;
            //}
            leftPower = -power;
            rightPower = power;
            robot.front_left.setPower(leftPower);
            robot.back_left.setPower(leftPower);
            robot.front_right.setPower(rightPower);
            robot.back_right.setPower(rightPower);

        }
        else if (degrees == 360 || degrees == -360 || degrees == 0){
            leftPower = power;
            rightPower = power;
            robot.front_left.setPower(leftPower);
            robot.back_left.setPower(leftPower);
            robot.front_right.setPower(rightPower);
            robot.back_right.setPower(rightPower);
            targetAngle = degrees;
        }
        else{
            return;
        }

        // set power to rotate.

        //left turn
        /*robot.front_left.setPower(0);  //negative
        robot.front_right.setPower(0);  //positive
        robot.back_left.setPower(0);    //negative
        robot.back_right.setPower(0);    //positive

         */

        // rotate until turn is completed.
        if (degrees <= 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.gyro.getHeading() == 0)
            {
                telemetry.addData("gyro heading", robot.gyro.getHeading());
                telemetry.update();
                //rotate(90,0.5);
                idle();
            }

            while (opModeIsActive() && robot.gyro.getHeading() > targetAngle)
            {
                telemetry.addData("gyro heading", gyro.getHeading());
                telemetry.update();
                idle();
            }
        }
        else
            while (opModeIsActive() && robot.gyro.getHeading() < targetAngle)
            {
                telemetry.addData("gyro heading", robot.gyro.getHeading());
                telemetry.update();
                //rotate(-90,0.5);
                idle();
            }

        // turn the motors off.
        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);


        // Reset gyro heading to zero on new direction we are now pointing.
        robot.gyro.resetZAxisIntegrator();
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

    public void intakecollect (double speed, double seconds){
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double milliseconds = seconds * 1000;
            double i = 0;
            while (opModeIsActive() && i < milliseconds) {

                robot.intake.setPower(speed);

                sleep(1);
                i++;
            }

            // Stop all motion;
            robot.intake.setPower(0);

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

}

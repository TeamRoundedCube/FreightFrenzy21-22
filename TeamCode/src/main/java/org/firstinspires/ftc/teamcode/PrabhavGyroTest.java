package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

    @Autonomous(name = "PrabhavGyroTest")
    @Disabled
public class PrabhavGyroTest extends LinearOpMode {


        FFHardwareFullBot robot = new FFHardwareFullBot();

        @Override
        public void runOpMode() {

            robot.init(hardwareMap);

            /** Wait for the game to begin */
            //telemetry.addData(">", "Press Play to start op mode");
            //telemetry.update();

            telemetry.addData("Mode", "starting gyro calibration...please wait");
            telemetry.update();
            //telemetry.addData("X value",rawX);
            //telemetry.addData("Y Value",rawY);
            //telemetry.addLine("Waiting for start");
            //telemetry.update();
          //  robot.gyro.calibrate();
            //robot.gyro.resetZAxisIntegrator();
            // make sure the gyro is calibrated before continuing
            while (!isStopRequested() && robot.gyro.isCalibrating()) {
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "gyro calibrated...waiting for start");
            telemetry.update();
            telemetry.addData("Status: ", robot.gyro.status());

            waitForStart();

            if (opModeIsActive()) {

                //robot.gyro.calibrate();
                rotate(90,0.5);


            }
        }
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
                while (degrees < 0){
                    targetAngle++;
                }
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
                while (degrees > 0){
                    targetAngle++;
                }
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
                    telemetry.addData("gyro heading", robot.gyro.getHeading());
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

    }
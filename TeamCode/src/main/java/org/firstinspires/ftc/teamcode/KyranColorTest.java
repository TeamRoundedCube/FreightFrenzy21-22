//  ____      ____      ____           ____   ______________              _______            ____        ___
// |    |    /    /     \   \        /    /  |     _______   \          /         \         |     \     |   |
// |    |   /    /       \   \     /    /    |   |        |   \        /    / \    \        |      \    |   |
// |    |  /    /         \   \  /    /      |   |_______|    /       /    /   \    \       |       \   |   |
// |    | /    /           \   \/   /        |     ___      _/       /    /     \    \      |    |\  \  |   |
// |    | \    \            |     |          |    |   \    \        /    /_______\    \     |    | \  \ |   |
// |    |  \    \           |     |          |    |    \    \      /    /_________\    \    |    |  \       |
// |    |   \    \          |     |          |    |     \    \    /    /           \    \   |    |   \      |
// |____|    \____\         |_____|          |____|      \____\  /____/             \____\  |____|    \_____|

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "KyranColorTest")
//@Disabled
public class KyranColorTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    KyranHardwareFullBot robot = new KyranHardwareFullBot();

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

        robot.init(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            //driveInches(0.3, 10);
            driveForward(0.3, 5);
            sleep(2000);
            driveReverse(0.5,5);



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

    public void driveReverse (double speed, double distance) {
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
                                    &&  runtime.seconds() < timeoutS)*/
            {
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

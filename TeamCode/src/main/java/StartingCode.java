import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.HardwareFullBot;

//edit by Prabhav 9/26/21 @ 9:39am
    //Comment added by Rushda to test on 9-25-21 at 7:34PM to test if push and commit works fine or not

    // Comment added by Vikrant to test on 9-26-2021 at 9:43AM to test if push and commit


   // Comment added by Rajiv J 9/26/2021 at 9:44am
@Autonomous(name="StartingCode")
@Disabled
public class StartingCode extends LinearOpMode {



       private ElapsedTime runtime = new ElapsedTime();
        HardwareFullBot robot = new HardwareFullBot();   // Use a Pushbot's hardware


        @Override
        public void runOpMode() {

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);
            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            //clawClose();
            //turnOnFlicker(0.38);
            //  turnOnFlicker(0.5);
            telemetry.addData("running", "running");
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {
                GoToCarousel();
                SpinCarousel();
                MoveBlockToSU();
            }

            sleep(1000);     // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
        public void StopRobot(){    //Set 4 Wheel Motor Powers To 0
            robot.front_left.setPower(0);
            robot.front_right.setPower(0);
            robot.back_left.setPower(0);
            robot.back_right.setPower(0);
        }
        public void GoToCarousel(){            // Robot Straiphs to the left for 5 seconds
            robot.front_right.setPower(-1);
            robot.back_right.setPower(-1);
            robot.front_left.setPower(1);
            robot.back_left.setPower(1);
            sleep(5000);
            StopRobot();
    }
        public void SpinCarousel(){          //Spin carousel wheel for 1.5 seconds to deliver duck
            robot.spincarousel.setPower(1);
            sleep(1500);
            robot.spincarousel.setPower(0);
            StopRobot();
        }
        public void MoveBlockToSU(){  //Move Block To Storage Unit
            robot.front_right.setPower(1);
            robot.front_left.setPower(1);
            robot.back_left.setPower(1);
            robot.back_right.setPower(1);
            sleep(2000);
        }
       // public void clawOpen() {
            //clawControl(0);
        //}
       // public void clawClose() {clawControl(1); }
        //public void clawControl(double position) {

            //robot.claw.setPosition(position);
            //sleep(2000);

        //}

}


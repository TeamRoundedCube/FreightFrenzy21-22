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

@Autonomous(name = "KyranWheelTest")
@Disabled
public class KyranWheelTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    FFHardwareFullBot robot = new FFHardwareFullBot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData(">", "started");
            telemetry.update();
            robot.front_right.setPower(1);
            robot.front_left.setPower(1);
            robot.back_right.setPower(1);
            robot.back_left.setPower(1);
            sleep(2000);
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.back_left.setPower(0);

        }
    }
}

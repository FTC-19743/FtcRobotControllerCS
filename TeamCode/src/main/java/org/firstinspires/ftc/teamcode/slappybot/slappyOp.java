package org.firstinspires.ftc.teamcode.slappybot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;



@TeleOp(name = "slappyOp", group = "LinearOpMode")
public class slappyOp extends LinearOpMode {
    TeamGamepad gamepad;
    WebcamName hibbleMotors;
    @Override

    public void runOpMode() {
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);


        VisionPortal visionPortal;
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortal = builder.build();

        waitForStart();

        while (opModeIsActive()) {

        }
        visionPortal.close();
    }
}

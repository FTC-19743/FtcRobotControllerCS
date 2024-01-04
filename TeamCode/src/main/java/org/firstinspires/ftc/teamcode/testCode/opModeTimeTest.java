package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@Autonomous(name = "opModeTimeTest", group = "LinearOpMode")
public class opModeTimeTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        teamUtil.init(this);

        waitForStart();
        while (opModeIsActive()) {
            teamUtil.log("Entering the op mode");
            while(opModeIsActive()){}
            teamUtil.log("Op mode is no longer active");
            teamUtil.pause(400);
        }
    }
}

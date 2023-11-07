package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@Autonomous(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        robot = new Robot();
        robot.initialize();
        teamUtil.pause(1000);
        robot.calibrate();
        robot.drive.initCV();
        robot.drive.runSideTeamPropFinderProcessor();
        while(!opModeIsActive()){
            telemetry.addLine("Path: "+robot.drive.findTeamPropProcesser.propPosition);
            telemetry.update();
        }
        waitForStart();
        robot.auto(2,false);
    }
}


package org.firstinspires.ftc.teamcode.judging;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "JudgingOpMode", group = "LinearOpMode")
public class JudgingOpMode extends LinearOpMode {
    TeamGamepad gamepad;
    Robot robot;

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        robot = new Robot();
        robot.initialize();
        robot.calibrate();

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if(gamepad.wasDownPressed()){
                robot.launcher.toggleRelease();
            }
            if(gamepad.wasAPressed()){
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.JUDGING_BLINKIN);
            }
            if(gamepad.wasBPressed()){
                robot.intake.startIntake();
                teamUtil.pause(5000);
                robot.intake.sweeper.setPower(0);
                robot.intake.kicker.setPower(.1);
                robot.intake.openLid();
            }
            if(gamepad.wasYPressed()){
                robot.output.goToScore(3,robot.output.GrabberRotatorLoad,robot.output.StraferLoad);
                robot.output.grabberRotater.setPosition(robot.output.GrabberRotatorLowerLimit);
                robot.output.grabberStrafer.setPosition(robot.output.StraferRight);
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.JUDGING_RIGHT);
                teamUtil.pause(2000);
                robot.output.grabberRotater.setPosition(robot.output.GrabberRotatorUpperLimit);
                robot.output.grabberStrafer.setPosition(robot.output.StraferLeft);
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.JUDGING_LEFT);
                teamUtil.pause(2000);
                robot.output.grabberRotater.setPosition(robot.output.GrabberRotatorLoad);
                robot.output.grabberStrafer.setPosition(robot.output.StraferLoad);
                teamUtil.pause(1000);
            }
            if(gamepad.wasXPressed()){
                robot.output.goToLoad();
                robot.lift.armUp();
                robot.launcher.toggleRelease();
            }
            if(gamepad.wasOptionsPressed()){
                robot.lift.stowArm();
            }
        }
    }
}


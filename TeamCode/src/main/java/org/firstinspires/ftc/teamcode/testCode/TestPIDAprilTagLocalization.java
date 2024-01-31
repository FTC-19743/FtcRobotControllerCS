package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.libs.MiniPID;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "TestPIDAprilTags")
@Disabled
public class TestPIDAprilTagLocalization extends LinearOpMode {

    Drive drive;
    TeamGamepad gamepad;

    MiniPID xControl = new MiniPID(.1,0,0); // x power will .1/cm offset
    MiniPID yControl = new MiniPID(0.5,0,0); // y power .05/cm offset

    public void initialize () {
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        drive = new Drive();
        drive.initalize(null);
        drive.initCV(true);
        // set up motors
    }

    public boolean driveToAprilTagOffsetPID(double initialVelocity, double initialDriveHeading, double robotHeading, double xOffset, double yOffset, long timeout) {
        teamUtil.log("PID Drive to April Tag Offset X: " + xOffset + " Y: "+ yOffset);
        boolean details = true;
        long timeOutTime = System.currentTimeMillis() + timeout;
        long aprilTagTimeoutTime = 0;
        float driftCms = 2;
        org.opencv.core.Point tagOffset = new org.opencv.core.Point();
        teamUtil.log("Continue on Initial Heading");
        drive.aprilTag.getFreshDetections();
        xControl.reset();
        yControl.reset();
        while (!drive.getRobotBackdropOffset(tagOffset,true) && teamUtil.keepGoing(timeOutTime)) {
            drive.driveMotorsHeadingsFR(initialDriveHeading, robotHeading, initialVelocity); // continue on initial heading until we see a tag
        }
        drive.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        teamUtil.log("Driving based on tags");
        double x,y,rotationAdjust;
        double cmsToTravel, cmsToStrafe, cmsToBackup;

        while (teamUtil.keepGoing(timeOutTime)) { // Use April Tags to go the rest of the way
            x = xControl.getOutput(xOffset, tagOffset.x); // TODO: Adjust signs on x and y if needed
            y = yControl.getOutput(yOffset, -tagOffset.y);
            cmsToStrafe = tagOffset.x - xOffset;
            cmsToBackup = tagOffset.y - yOffset;
            cmsToTravel = Math.sqrt(cmsToStrafe * cmsToStrafe + cmsToBackup * cmsToBackup);
            if (Math.abs(cmsToTravel) < driftCms) {
                break;
            }
            double headingError = drive.getHeadingError(robotHeading); // Difference between desired and actual robot heading
            rotationAdjust = .02f * headingError; // scale based on amount of rotational error
            if (details)
                teamUtil.log("strafe: " + cmsToStrafe + " back: " + cmsToBackup + " travel: " + cmsToTravel + " headingError: " + headingError + " x: " + x + " y: " + y + " rot: " + rotationAdjust);

            // x, y, rotation input mixing
            drive.fl.setPower(x + y + rotationAdjust);
            drive.bl.setPower(x - y + rotationAdjust);
            drive.fr.setPower(x - y - rotationAdjust);
            drive.br.setPower(x + y - rotationAdjust);

            aprilTagTimeoutTime = System.currentTimeMillis() + 1000;
            while (!drive.getRobotBackdropOffset(tagOffset,false) && teamUtil.keepGoing(aprilTagTimeoutTime)) {
                if (details) teamUtil.log("WARNING: Lost sight of tags!");
                // TODO: keep updating motors based on where we think we are?
                // TODO: could just presume we have traveled the same distance in both x and y as we did last time
            }
        }
        drive.stopMotors();
        drive.setMotorsWithEncoder();
        if (System.currentTimeMillis() > timeOutTime || System.currentTimeMillis() > aprilTagTimeoutTime) {
            teamUtil.log("driveToAprilTagOffsetPID - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("Drive to April Tag OffsetPID - FINISHED");
            return true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        drive.switchCV(Drive.cvCam.REAR_APRILTAG);
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            gamepad.loop();
            if (gamepad.wasYPressed()) {
                drive.setHeading(180);
                timer.reset();
                driveToAprilTagOffsetPID(1500,270,180,0,20,4000);
                teamUtil.log("Elapsed Time:" + timer.seconds());
            }
            if (gamepad.gamepad.left_bumper) {
                if(gamepad.wasUpPressed()){
                    xControl.setP(xControl.getP()+.05);
                }else if(gamepad.wasDownPressed()){
                    xControl.setP(xControl.getP()-.05);
                }else if(gamepad.wasLeftPressed()){
                    yControl.setP(yControl.getP()+.05);
                } else if(gamepad.wasRightPressed()){
                    yControl.setP(yControl.getP()-.05);
                }
            }
            telemetry.addLine("x: P:" + xControl.getP() + "I:" + xControl.getI() + "D:" + xControl.getD());
            telemetry.addLine("y: P:" + yControl.getP() + "I:" + yControl.getI() + "D:" + yControl.getD());
            telemetry.update();
        }
    }
}

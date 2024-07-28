package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "testNewDrive", group = "LinearOpMode")

public class testNewDrive extends LinearOpMode {
    TeamGamepad gamepad;
    Drive drive;
    //@Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        drive = new Drive();
        drive.initalize();
        drive.setHeading(180);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        drive.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            drive.newDriveMotorTelemetry();
            if (gamepad.gamepad.right_stick_button && gamepad.gamepad.left_stick_button) {
                drive.setHeading(0);
            }
            if (gamepad.wasHomePressed()){

            }
            double rotateAdjustment;
            if(Math.abs(gamepad1.right_stick_x)<.9){
                rotateAdjustment=.5;
            }else if(Math.abs(gamepad1.right_stick_x)<.1){
                rotateAdjustment=0;

            }else{rotateAdjustment=1;

            }
            double forwardAdjustment;
            if(Math.abs(gamepad1.left_stick_y)<.9){
                forwardAdjustment=.5;
            }else if(Math.abs(gamepad1.left_stick_y)<.1){
                forwardAdjustment=0;

            }else{
                forwardAdjustment=1;

            }
            double strafeAdjustment;
            if(Math.abs(gamepad1.left_stick_x)<.9){
                strafeAdjustment=.5;
            }else if(Math.abs(gamepad1.left_stick_x)<.1){
                strafeAdjustment=0;

            }else{
                strafeAdjustment=1;

            }
            double y = -gamepad1.left_stick_y*forwardAdjustment; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x*strafeAdjustment;
            double rx = gamepad1.right_stick_x*rotateAdjustment;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.



            // Rotate the movement direction counter to the bot's rotation
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



            drive.fl.setPower(frontLeftPower);
            drive.fr.setPower(backLeftPower);
            drive.bl.setPower(frontRightPower);
            drive.br.setPower(backRightPower);

            telemetry.addLine("fr: " + drive.fr.getPower());
            telemetry.addLine("fl: " + drive.fl.getPower());
            telemetry.addLine("br: " + drive.br.getPower());
            telemetry.addLine("bl: " + drive.bl.getPower());
            telemetry.addLine("leftx: " + gamepad1.left_stick_x);
            telemetry.addLine("lefty: " + gamepad1.left_stick_y);
            telemetry.addLine("rightx: " + gamepad1.right_stick_x);



            telemetry.update();
        }
    }
}

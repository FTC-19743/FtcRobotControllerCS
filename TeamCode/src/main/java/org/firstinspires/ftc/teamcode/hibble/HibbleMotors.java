package org.firstinspires.ftc.teamcode.hibble;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class HibbleMotors {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public DcMotorEx motorZero;
    public DcMotorEx motorOne;

    public HibbleMotors() {
        teamUtil.log("Constructing Hibble Motors");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;

    }

    public void init() {
        teamUtil.log("Initializing Arm");
        motorZero = hardwareMap.get(DcMotorEx.class, "motor_zero");
        motorOne = hardwareMap.get(DcMotorEx.class, "motor_one");
        motorZero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveMotors(double power){
        motorZero.setPower(power);
        motorOne.setPower(power);
    }

}

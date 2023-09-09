package org.firstinspires.ftc.teamcode.assemblies.OLD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.libs.teamUtil;


public class IntakeOutputWheel {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    HardwareMap hardwareMap;


    private DcMotor intakeOutputMotor = null;

    public IntakeOutputWheel(){
        teamUtil.log("Constructing Intake Output Wheel");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
    }

    public void initialize() {

        teamUtil.log("Initializing Intake");

        intakeOutputMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeOutputMotor.setDirection(DcMotor.Direction.FORWARD);





    }
    public void intakeGo(double speed){
        intakeOutputMotor.setPower(speed);
        }


    public void intakeStop(){
        intakeOutputMotor.setPower(0);
    }

    public void intakeRunUntilResistance(double speed){
        log("Empty For Now");
    }
    public void outputGo(double speed){
        intakeOutputMotor.setPower(speed);
    }


    public void outputStop(){
        intakeOutputMotor.setPower(0);
    }


    public void intakeOutputControl(float speed) {
        double motorPower;


        motorPower = Range.clip(speed, -1.0, 1.0);


        intakeOutputMotor.setPower(motorPower);

    }




}

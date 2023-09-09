package org.firstinspires.ftc.teamcode.assemblies.OLD;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
public class OutakeSlide {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo outakeSlider;
    public static double MaxSliderPosition = 0.315;
    public static double MinSliderPosition = .51;
    public static double SliderIncrements = .005; // might need to be changed


    //placeholders
    //public static double Level1;
    //public static double Level2;
    //public static double Level3;

    public OutakeSlide() {
        teamUtil.log("Constructing Outake Arm");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void init() {
        teamUtil.log("Initializing Arm");
        outakeSlider = hardwareMap.get(Servo.class, "outake_slider");

    }

    public void writeTelemetry(){
        telemetry.addData("Outake Slider","Slider Position:%f",outakeSlider.getPosition());
    }



    public void resetSlider(){
        outakeSlider.setPosition(0);
    }

    public void runSliderToPosition(double position){
        outakeSlider.setPosition(position);
    }

}
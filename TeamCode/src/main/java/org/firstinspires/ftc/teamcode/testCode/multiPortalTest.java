
package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
@Disabled
public class multiPortalTest extends LinearOpMode
{
    TeamGamepad tgp = new TeamGamepad();
    VisionPortal portal1, portal2, portal3;
    AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setMsTransmissionInterval(50);
        teamUtil.init(this);
        tgp.initilize(true);

        aprilTag = new AprilTagProcessor.Builder().build();

        int[] viewIDs = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.HORIZONTAL);

        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Rear"))
                .setLiveViewContainerId(viewIDs[0])
                .addProcessor(aprilTag)
                //.enableLiveView(false)
                //.setAutoStopLiveView(true)
                .build();

        while (!isStopRequested() && portal1.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 1 to come online");
            telemetry.update();
        }
        portal1.stopStreaming();
        while (!isStopRequested() && portal1.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY)
        {
            telemetry.addLine("Waiting for portal 1 to stop");
            telemetry.update();
        }
        if (isStopRequested())
        {
            return;
        }

        portal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Front"))
                .setLiveViewContainerId(viewIDs[1])
                //.enableLiveView(true)
                //.setAutoStopLiveView(true)
                .build();

        while (!isStopRequested() && portal2.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 2 to come online");
            telemetry.update();
        }
        portal2.stopStreaming();
        while (!isStopRequested() && portal2.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY)
        {
            telemetry.addLine("Waiting for portal 2 to stop");
            telemetry.update();
        }
        if (isStopRequested())
        {
            return;
        }

        portal3 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam Right"))
                .setLiveViewContainerId(viewIDs[2])
                //.enableLiveView(true)
                //.setAutoStopLiveView(true)
                .build();

        while (!isStopRequested() && portal3.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addLine("Waiting for portal 3 to come online");
            telemetry.update();
        }
        portal3.stopStreaming();
        while (!isStopRequested() && portal3.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY)
        {
            telemetry.addLine("Waiting for portal 3 to stop");
            telemetry.update();
        }
        if (isStopRequested())
        {
            return;
        }

        while (!isStopRequested() && !opModeIsActive())
        {
            telemetry.addLine("All cameras online");
            telemetry.update();
            sleep(500);
        }
        long last = 0, now = 0;
        while (opModeIsActive()) {
            now = System.currentTimeMillis();
            if (now-last>10) {
                teamUtil.log("LONG Loop: "+ (now-last));
            }
            last = now;
            tgp.loop();
            telemetry.addLine("Rear: "+ portal1.getCameraState());
            telemetry.addLine("Side: "+ portal3.getCameraState());
            telemetry.addLine("Front: "+ portal2.getCameraState());
            telemetry.addLine("Tags: "+ aprilTag.getDetections());


            telemetry.update();
            if (tgp.wasYPressed()) {
                if (portal1.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                    stopStreaming();
                    portal1.resumeStreaming();
                }
            }
            if (tgp.wasXPressed()) {
                portal1.stopStreaming();
            }
            if (tgp.wasBPressed()) {
                if (portal2.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                    stopStreaming();
                    portal2.resumeStreaming();
                }
            }
            if (tgp.wasAPressed()) {
                portal2.stopStreaming();
            }
            if (tgp.wasUpPressed()) {
                if (portal3.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
                    stopStreaming();
                    portal3.resumeStreaming();
                }
            }
            if (tgp.wasLeftPressed()) {
                portal3.stopStreaming();
            }
            if (tgp.wasDownPressed()) {
                stopStreaming();
            }
        }
    }
    public void stopStreaming() {
        if (portal1.getCameraState() == VisionPortal.CameraState.STREAMING) {
            portal1.stopStreaming();
        }
        if (portal2.getCameraState() == VisionPortal.CameraState.STREAMING) {
            portal2.stopStreaming();
        }
        if (portal3.getCameraState() == VisionPortal.CameraState.STREAMING) {
            portal3.stopStreaming();
        }
    }
}

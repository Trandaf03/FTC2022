package org.firstinspires.ftc.teamcode.utilities.rec;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "recunoastere")
public class recAutoMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        recunoastere detector = new recunoastere(telemetry);
        camera.setPipeline(detector);
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            switch (detector.getLocation()) {
                case LEFT:
                    telemetry.addData("Pozitie rata", "stanga");
                    break;
                case CENTER:
                    telemetry.addData("Pozitie rata", "centru");
                    break;
                case RIGHT:
                    telemetry.addData("Pozitie rata", "dreapta");
                    break;
                case NOT_FOUND:
                    telemetry.addData("Pozitie rata", "n-am gasit rata");
            }

        }
        camera.stopStreaming();
    }
}

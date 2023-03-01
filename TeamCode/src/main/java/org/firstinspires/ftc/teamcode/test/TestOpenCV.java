package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
@Disabled
public class TestOpenCV extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline());

        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        waitForStart();
        while (opModeIsActive()) {

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            telemetry.addData("Frame count", webcam.getFrameCount());
            telemetry.addData("FPS", webcam.getFps());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.update();
        }
    }
}

class SamplePipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols() / 4.0,
                        input.rows() / 4.0
                ),
                new Point(
                        input.cols() * 3.0/4.0,
                        input.rows() * 3.0/4.0
                ),
                new Scalar(0, 255, 0),
                4
        );


        return input;
    }
}
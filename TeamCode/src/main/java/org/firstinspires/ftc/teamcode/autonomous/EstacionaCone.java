package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class EstacionaCone extends LinearOpMode {
    private OpenCvCamera camera;
    private AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() throws InterruptedException {

        // configurando o hardware
        DestemidosBot robot = new DestemidosBot(hardwareMap);
        AutonomoSystem driveAuto = new AutonomoSystem(robot.drivetrain, robot.localizationSystem, robot.voltageSensor);

        // configurando camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        // configurando a pipeline
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(
                RobotConstants.OPENCV_tagsize,
                RobotConstants.OPENCV_fx,
                RobotConstants.OPENCV_fy,
                RobotConstants.OPENCV_cx,
                RobotConstants.OPENCV_cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        }

        final TrajectorySequence frente = driveAuto.trajectorySequenceBuilder(new Pose2d())
            .forward(20)
            .build();

        waitForStart();

        // loop de init
        // aqui garantimos que pelo menos a imagem foi reconhecida
        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    switch (tag.id) {
                        case RobotConstants.IMAGEM_1:
                        case RobotConstants.IMAGEM_2:
                        case RobotConstants.IMAGEM_3:
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                    }
                }
                if(tagFound)
                {
                    robot.setHubColor(Color.RED);
                    telemetry.addData("TAG FOUND!:", tagOfInterest.id);
                }
                else
                {
                    telemetry.addLine("Tag não encontrada");

                    if(tagOfInterest != null)
                    {
                        telemetry.addLine("\n A Tag já foi vista anteriormente!");
                    }
                }

            }
            telemetry.update();
        }

        final TrajectorySequence regiao1 = driveAuto.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(42)
                .forward(42)
                .build();

        final TrajectorySequence regiao2 = driveAuto.trajectorySequenceBuilder(new Pose2d())
                .forward(42)
                .build();

        final TrajectorySequence regiao3 = driveAuto.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(42)
                .forward(42)
                .build();

        waitForStart();
        camera.stopStreaming();
        camera.closeCameraDevice();

        // movemos para a região sorteada
        switch (tagOfInterest.id) {
            case RobotConstants.IMAGEM_1:
                driveAuto.followTrajectorySequence(regiao1);
                break;
            case RobotConstants.IMAGEM_2:
                driveAuto.followTrajectorySequence(regiao2);
                break;
            case RobotConstants.IMAGEM_3:
                driveAuto.followTrajectorySequence(regiao3);
                break;
        }
        terminateOpModeNow();
    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Config
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

        // começa a stream da câmera pro ftc-dashboard
        FtcDashboard.getInstance().startCameraStream(camera, 60);

        camera.setPipeline(aprilTagDetectionPipeline);
        //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(
                        RobotConstants.resolutionWidth,
                        RobotConstants.resolutionHeight,
                        OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        final TrajectorySequence frente = driveAuto.trajectorySequenceBuilder(new Pose2d())
            .forward(20)
            .build();

        final TrajectorySequence regiao1 = driveAuto.trajectorySequenceBuilder(frente.end())
                .strafeRight(42)
                .forward(42)
                .build();

        final TrajectorySequence regiao2 = driveAuto.trajectorySequenceBuilder(frente.end())
                .forward(42)
                .build();

        final TrajectorySequence regiao3 = driveAuto.trajectorySequenceBuilder(frente.end())
                .strafeLeft(42)
                .forward(42)
                .build();

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

        waitForStart();

        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        driveAuto.followTrajectorySequence(frente);

        //camera.stopStreaming();
        camera.closeCameraDevice();

        if(tagOfInterest == null){
            //driveAuto.followTrajectorySequence();
        }

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
        //terminateOpModeNow();
    }
}

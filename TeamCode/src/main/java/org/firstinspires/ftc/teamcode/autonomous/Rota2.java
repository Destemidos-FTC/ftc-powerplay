package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Autonomous(name = "Rota2")
public class Rota2 extends OpMode {
    private OpenCvCamera camera;
    private AprilTagDetection tagOfInterest = null;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // configurando o hardware
    private final DestemidosBot robot = new DestemidosBot(hardwareMap);
    private final AutonomoSystem driveAuto = new AutonomoSystem(
            robot.drivetrain,
            robot.localizationSystem,
            robot.voltageSensor);
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

    @Override
    public void init() {
        // configurando camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        // configurando a pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(
                RobotConstants.OPENCV_tagsize,
                RobotConstants.OPENCV_fx,
                RobotConstants.OPENCV_fy,
                RobotConstants.OPENCV_cx,
                RobotConstants.OPENCV_cy);

        // começa a stream da câmera pro ftc-dashboard
        FtcDashboard.getInstance().startCameraStream(camera, 60);

        // define o pipeline e inicia a câmera
        camera.setPipeline(aprilTagDetectionPipeline);
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
    }

    @Override
    public void init_loop() {
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

    @Override
    public void start() {

        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        driveAuto.followTrajectorySequence(frente);
        
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

        terminateOpModeNow();
    }

    @Override
    public void loop() {

    }
}

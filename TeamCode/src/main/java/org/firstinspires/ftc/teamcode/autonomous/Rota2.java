package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
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

    private DestemidosBot robot;

    boolean tagFound = false;

    // configurando o hardware
    private AutonomoSystem driveAuto;

    Command ArmToHighJunction;

    TrajectorySequence ajuste;

    TrajectorySequence ajuste2;

    TrajectorySequence cone1;

    TrajectorySequence cone2;

    TrajectorySequence regiao1;

    TrajectorySequence regiao2;

    TrajectorySequence regiao3;

    @Override
    public void init() {

        // habilita a telemetria no ftc-dashboard
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        // inicializa os sistemas
        robot = new DestemidosBot(hardwareMap);
        driveAuto = new AutonomoSystem(
                robot.drivetrain,
                robot.localizationSystem,
                robot.voltageSensor);

        // criando as trajetórias
        ajuste = driveAuto.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .strafeLeft(18)
                .build();

        ajuste2 = driveAuto.trajectorySequenceBuilder(ajuste.end())
                .strafeRight(18)
                .build();

        cone1 = driveAuto.trajectorySequenceBuilder(ajuste2.end())
                .forward(62)
                .splineToLinearHeading(new Pose2d(68, 0, Math.toRadians(40)), 0)
                .addDisplacementMarker(() -> {
                    new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.HIGH));
                    new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.HIGH));
                    CommandScheduler.getInstance().run();
                })
                .build();

        cone2 = driveAuto.trajectorySequenceBuilder(cone1.end())
                .strafeRight(55)
                .build();

        regiao1 = driveAuto.trajectorySequenceBuilder(ajuste.end())
                .strafeRight(55)
                .forward(42)
                .build();

        regiao2 = driveAuto.trajectorySequenceBuilder(ajuste.end())
                .forward(42)
                .build();

        regiao3 = driveAuto.trajectorySequenceBuilder(ajuste.end())
                .strafeLeft(55)
                .forward(42)
                .build();

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(robot.armSystem, robot.gripper);

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

        // definindo a posição inicial como (0,0,0) pro roadrunner
        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
    }

    @Override
    public void init_loop() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
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


        if(tagOfInterest == null){
            driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
            driveAuto.followTrajectorySequence(ajuste);

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    switch (tag.id) {
                        case RobotConstants.IMAGEM_1:
                        case RobotConstants.IMAGEM_2:
                        case RobotConstants.IMAGEM_3:
                            tagOfInterest = tag;
                            tagFound = true;
                            robot.setHubColor(Color.RED);
                            break;
                    }
                }
            }
            driveAuto.followTrajectorySequence(ajuste2);
        }
        else{
            camera.closeCameraDevice();
        }

        camera.closeCameraDevice();
        driveAuto.followTrajectorySequence(cone1);




        /*Estacionar
        switch (tagOfInterest.id){
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

         */





        terminateOpModeNow();
    }

    @Override
    public void loop() {

    }
}

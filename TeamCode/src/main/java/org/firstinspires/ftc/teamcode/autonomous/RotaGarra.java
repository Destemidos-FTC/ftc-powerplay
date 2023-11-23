package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@Autonomous(name = "RotaGarra")
public class RotaGarra extends OpMode {
    //   private AprilTagDetection tagOfInterest = null;
    // private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DestemidosBot robot;

    boolean tagFound = false;



    // configurando o hardware
    private AutonomoSystem driveAuto;

    TrajectorySequence ajuste;
    TrajectorySequence esque;
    TrajectorySequence spikeMeio2;
    TrajectorySequence spikeMeio;
    TrajectorySequence spikeMeio3;
    TrajectorySequence meio;
    TrajectorySequence meio2;

    ElapsedTime timer;





    @Override
    public void init() {

        // habilita a telemetria no ftc-dashboard
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        // inicializa os sistemas
        timer = new ElapsedTime();
        robot = new DestemidosBot(hardwareMap);
        driveAuto = new AutonomoSystem(
                robot.drivetrain,
                robot.localizationSystem,
                robot.voltageSensor);

        // reseta o agendador de comandos
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(robot.armSystem, robot.servo);

        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

         ajuste = driveAuto.trajectorySequenceBuilder(new Pose2d())
                 .forward(32)
                 .build();

         esque = driveAuto.trajectorySequenceBuilder( ajuste.end())
                .back(21)
                .turn(Math.toRadians(-220))
                .back(115)
                .strafeRight(30)
                .build();

         spikeMeio2 = driveAuto.trajectorySequenceBuilder( esque.end())
                .turn(Math.toRadians(-45))
                .build();

         spikeMeio = driveAuto.trajectorySequenceBuilder( spikeMeio2.end())
                .addDisplacementMarker(() -> {
                    robot.servo.monhecaServoRotation(80);
                })
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    timer.reset();
                    while (timer.seconds() < 2) {
                        robot.servo.wristServoA.setPower(-1);
                        robot.servo.wristServoB.setPower(-1);
                    }
                })
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    robot.servo.monhecaServoRotation(-200);
                })
                .waitSeconds(1)
                .build();

         spikeMeio3 = driveAuto.trajectorySequenceBuilder( spikeMeio.end())
                 .back(28)
                 .turn(Math.toRadians(-315))
                 .back(110)
                 .strafeRight(30)
                 .build();

         meio = driveAuto.trajectorySequenceBuilder(spikeMeio3.end())
                 .turn(Math.toRadians(45))
                 .forward(10)
                 .build();

         meio2 = driveAuto.trajectorySequenceBuilder(meio.end())
                 .back(10)
                 .build();




        // configurando camera
        /*
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
         */

        // definindo a posição inicial como (0,0,0) pro roadrunner
        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
    }

    @Override
    public void start() {

        driveAuto.followTrajectorySequence(ajuste);
        //while (true)
        // if (sensor color)
        //if (sensor color)
        driveAuto.followTrajectorySequence(meio);

        driveAuto.followTrajectorySequence(spikeMeio);

        driveAuto.followTrajectorySequence(meio2);

        driveAuto.followTrajectorySequence(spikeMeio2);

        driveAuto.followTrajectorySequence(spikeMeio3);

























    }

    @Override
    public void loop() {

    }


    }

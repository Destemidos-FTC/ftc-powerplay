package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Rota2")
public class Rota2 extends OpMode {
    private AprilTagDetection tagOfInterest = null;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DestemidosBot robot;

    boolean tagFound = false;


    // configurando o hardware
    private AutonomoSystem driveAuto;

    TrajectorySequence ajuste;
    TrajectorySequence veryEsq;
    TrajectorySequence veryDir;
    TrajectorySequence very180;
    TrajectorySequence esque;






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

        // reseta o agendador de comandos
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(robot.armSystem, robot.servo);

        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

         ajuste = driveAuto.trajectorySequenceBuilder(new Pose2d())
                 .forward(32)
                 .build();

         veryEsq = driveAuto.trajectorySequenceBuilder( ajuste.end())
                .turn(Math.toRadians(-180))
                .build();

         veryDir = driveAuto.trajectorySequenceBuilder( ajuste.end())
                .turn(Math.toRadians(180))
                .build();


         very180 = driveAuto.trajectorySequenceBuilder( veryEsq.end())
                .turn(Math.toRadians(360))
                .turn(Math.toRadians(70))
                .build();

         esque = driveAuto.trajectorySequenceBuilder( very180.end())
                .turn(Math.toRadians(-180))
                .back(28)
                .turn(Math.toRadians(195))
                .forward(80)
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

        driveAuto.followTrajectorySequence(veryEsq);

        driveAuto.followTrajectorySequence(very180);

        driveAuto.followTrajectorySequence(esque);

















    }

    @Override
    public void loop() {

    }

}

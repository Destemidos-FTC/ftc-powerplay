package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;

@Autonomous
public class Rota1A extends LinearOpMode {
    public static int NUM_COLLECT_CYCLES = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /**
         * definimos a posição inicial do robô aqui
         */
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

        /**
         * vai em direção ao tatame de frente a pilha de cones
         * entrga o cone precarregado na junction alta
         * segue em direção a pilha de cones para a coleta
        */
        final TrajectorySequence etapa1 = drive.trajectorySequenceBuilder(new Pose2d(35, 65, Math.toRadians(270)))

                // vai até o meio do 3° tatame
                .forward(45)

                // faz uma curva até a junção alta
                .splineToLinearHeading(new Pose2d(32, 9, Math.toRadians(220)), Math.toRadians(180))

                // entrega o cone na junção alta
                .addDisplacementMarker(()->{

                })
                .build();

        final TrajectorySequence etapa2 = drive.trajectorySequenceBuilder(etapa1.end())
                // vai pra pilha de cones
                .splineToLinearHeading(new Pose2d(40, 13.2, Math.toRadians(0)), Math.toRadians(90))
                .forward(18)

                // coleta o cone
                .addDisplacementMarker(() -> {
                    // entrega o cone
                })

                //
                .back(18)
                .splineToLinearHeading(new Pose2d(32.5, 11, Math.toRadians(230)), Math.toRadians(180))
                .build();

        waitForStart();

        // 1° passo
        drive.followTrajectorySequence(etapa1);
    }
}

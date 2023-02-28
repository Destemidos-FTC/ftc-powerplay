package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;

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
        final TrajectorySequence entrega_do_cone = drive.trajectorySequenceBuilder( new Pose2d(0.0, 0.0, 0.0))
                .forward(60)
                .turn(Math.toRadians(125.0))
                .addDisplacementMarker(() -> {
                    // entrega o cone
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(55, 5, Math.toRadians(90)),  Math.toRadians(0))
                .build();

        final TrajectorySequence ciclo_de_entregas = drive.trajectorySequenceBuilder(entrega_do_cone.end())
                .addDisplacementMarker(() ->{
                    // coleta o cone na pilha
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(60, 0, Math.toRadians(90)),  Math.toRadians(0))
                .build();

        /**
         * (ramalho): nesse caso, a localização vai variar com o que é sorteado no cone
         * por enquanto, vou deixar na localizção 2
         */
        final TrajectorySequence estacionar_local_sorteado = drive.trajectorySequenceBuilder(ciclo_de_entregas.end())
                .strafeLeft(30)
                .build();

        waitForStart();

        // 1° passo
        drive.followTrajectorySequence(entrega_do_cone);

        // 2° passo
        for(int i = 0; i <= NUM_COLLECT_CYCLES; i++) {
            drive.followTrajectorySequence(ciclo_de_entregas);
        }

        // 3° passo
        drive.followTrajectorySequence(estacionar_local_sorteado);
    }
}

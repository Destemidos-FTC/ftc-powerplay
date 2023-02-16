package com.destemidos.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        // Declare our first bot
        RoadRunnerBotEntity destemidosBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(15.748, 15.748)
                .setConstraints(80, 60, Math.toRadians(360), Math.toRadians(360), 13.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                                .forward(60)
                                .turn(Math.toRadians(125.0))
                                .addDisplacementMarker(() -> {
                                    // entrega o cone
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(55, 5, Math.toRadians(90)),  Math.toRadians(0))

                                // essa etapa se repete 5 vezes
                                .addDisplacementMarker(() ->{
                                    // coleta o cone na pilha
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(60, 0, Math.toRadians(90)),  Math.toRadians(0))

                                // finaliza estacionando na localização 2
                                .strafeLeft(25)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(destemidosBot)
                .start();
    }
}
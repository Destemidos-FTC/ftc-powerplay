package com.destemidos.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity destemidosBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(13.2, 15.75)
                .setConstraints(60, 60, 60, 60, 11.64653)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 65, Math.toRadians(270)))

                                // vai em direção a jumção alta e "cola" na frente dela
                                .forward(45)
                                .splineTo(new Vector2d(32, 7), Math.toRadians(225))

                                // coloca o cone
                                .addTemporalMarker(5, ()->{

                                })

                                // vai em direção a pilha de cones
                                .splineToLinearHeading(new Pose2d(45, 12, Math.toRadians(0)), Math.toRadians(0))
                                .forward(14)

                                // pega o cone
                                .addDisplacementMarker(()->{

                                })
                                // volta e "cola" na junção média
                                .back(21)
                                .turn(-Math.toRadians(49))
                                .forward(5)

                                // volta pra pilha e repete
                                .back(7)
                                .turn(Math.toRadians(49))
                                .forward(5)
                                .back(5)

                                // estaciona na região 1
                                .back(15)
                                .splineTo(new Vector2d(12, 35), Math.toRadians(90))
                                .turn(Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(destemidosBot)
                .start();
    }
}
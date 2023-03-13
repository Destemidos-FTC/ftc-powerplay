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
        MeepMeep meepMeep = new MeepMeep(400);

        // Declare our first bot
        RoadRunnerBotEntity destemidosBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(13.2, 15.75)
                .setConstraints(40, 35, Math.toRadians(60), Math.toRadians(60), 11.64653)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 65, Math.toRadians(270)))

                                // vai até o meio do 3° tatame
                                .forward(50)

                                // faz uma curva até a junção alta
                                .splineToLinearHeading(new Pose2d(29, 6, Math.toRadians(220)), Math.toRadians(180))

                                // entrega o cone na junção alta
                                .addDisplacementMarker(()->{

                                })

                                // vai pra pilha de cones
                                .splineToLinearHeading(new Pose2d(43, 13.2, Math.toRadians(0)), Math.toRadians(90))
                                .forward(19.2)

                                // coleta o cone
                                .addDisplacementMarker(() -> {
                                    // entrega o cone
                                })

                                //
                                .back(19.2)
                                .splineToLinearHeading(new Pose2d(29, 4, Math.toRadians(220)), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(destemidosBot)
                .start();
    }
}
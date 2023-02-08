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
                .setConstraints(33, 30, Math.toRadians(137), Math.toRadians(137), 13.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 10, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(60, 30, Math.toRadians(90)), Math.toRadians(90))
                                .setReversed(true)
                                //.splineToLinearHeading(new Pose2d(-34, -7, Math.toRadians(220)), Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(destemidosBot)
                .start();
    }
}
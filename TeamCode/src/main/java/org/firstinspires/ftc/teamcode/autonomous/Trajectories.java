package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequenceBuilder;

public final class Trajectories {

    // essa função é um helper na criação das trajetórias
    // já carrega por padrão todas as constantes do robô
    private static TrajectorySequenceBuilder buildTrajectory(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                RobotConstants.VEL_CONSTRAINT, RobotConstants.ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    /** NOTE(ramalho) Essa é a primeira trajetória a ser chamada
     * o objetivo dela é marcar o cone pré-instalado na junção alta mais próxima
     * que fica a 3 tatâmes à frente.
     */
    public static final TrajectorySequence Cone1 = buildTrajectory(new Pose2d())
            .forward(62)
            .turn(Math.toRadians(48))
            .build();

    public final TrajectorySequence regiao1 = buildTrajectory(new Pose2d())
            .strafeRight(55)
            .forward(42)
            .build();

    public final TrajectorySequence regiao2 = buildTrajectory(new Pose2d())
            .forward(42)
            .build();

    public final TrajectorySequence regiao3 = buildTrajectory(new Pose2d())
            .strafeLeft(55)
            .forward(42)
            .build();

}

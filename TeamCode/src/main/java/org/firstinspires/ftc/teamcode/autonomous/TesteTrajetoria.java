package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

@Autonomous(name = "TesteTrajetoria", group = "Test")
public class TesteTrajetoria extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence ir_para_pilha = drive.trajectorySequenceBuilder( new Pose2d(0.0, 0.0, 0.0))
                .setVelConstraint(RobotConstants.ULTRA_VEL_CONSTRAINT)
                .setAccelConstraint(RobotConstants.ULTRA_ACCEL_CONSTRAINT)
                .forward(60)
                .turn(Math.toRadians(125.0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // se direciona para a pilha de cones
        drive.followTrajectorySequence(ir_para_pilha);

        // volta de ré pra a juntion alta mais próxima
        //drive.followTrajectory(ir_para_junction);

        while (!isStopRequested() && opModeIsActive()) ;
    }
}

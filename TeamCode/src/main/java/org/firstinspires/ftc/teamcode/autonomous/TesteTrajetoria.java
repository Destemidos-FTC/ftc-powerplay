package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.drive.SampleMecanumDrive;

@Autonomous(name = "TesteTrajetoria", group = "Test")
public class TesteTrajetoria extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory curva_s_indo = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(80, 45), 0)
                .build();

        Trajectory curva_s_voltano = drive.trajectoryBuilder(curva_s_indo.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // se direciona aproximadamente 2 tatames em X e Y
        drive.followTrajectory(curva_s_indo);

        // percorre o caminho voltano, só que de costas
        drive.followTrajectory(curva_s_voltano);

        while (!isStopRequested() && opModeIsActive()) ;
    }
}

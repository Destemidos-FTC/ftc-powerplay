package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

/**
 * OpMode focado em experimentação de novas idéias
 * e de medição das informações do robô
 */
@TeleOp(name="TESTBOT", group = "Test")
public class FTC_TESTBOT extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // setup the hardware components
        final DestemidosBot robot = new DestemidosBot(hardwareMap);
        final Outtake outtake = new Outtake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.setBulkReadToAuto();

        waitForStart();
        while(opModeIsActive()) {

            // movimentação padrão das partidas
            robot.drivetrain.standardMecanumController(gamepad1);

            outtake.leftOuttake.setPower(gamepad2.left_stick_y / 2);
            outtake.rightOuttake.setPower(gamepad2.left_stick_y / 2);
        }
    }
}
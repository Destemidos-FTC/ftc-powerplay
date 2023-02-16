package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

/**
 * OpMode princiapl para as competições.
 * O foco é sempre buscar o máximo de performance
 * e estabilidade por aqui
 */
@TeleOp(name = "FTC_4", group = "TeleOps")
public class FTC_4 extends LinearOpMode {
    @Override
    public void runOpMode() {
        final DestemidosBot robot = new DestemidosBot(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            robot.setBulkReadToAuto();

            // movimentação padrão das partidas
            robot.drivetrain.standardMecanumController(gamepad1);

            // controles do braço e da mão
            robot.armSystem.moveArms(gamepad2);
        }
    }
}
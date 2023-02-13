package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.GripSystem;


@TeleOp(name = "FTC_4", group = "TeleOps")
public class FTC_4 extends LinearOpMode {
    private DestemidosBot robot;

    @Override
    public void runOpMode() {
        robot = new DestemidosBot(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            robot.setBulkReadToAuto();

            // movimentação padrão das partidas
            robot.drivetrain.driveStandardMecanum(gamepad1);

            // controles do braço e da mão
            ArmSystem.movimentarBraço(gamepad2, robot);

            GripSystem.coletarCones(gamepad2, robot);
        }
    }
}
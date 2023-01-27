package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

@TeleOp(name="TESTBOT", group = "Test")
public class FTC_TESTBOT extends LinearOpMode {
    private DestemidosBot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        // setup the hardware components
        robot = new DestemidosBot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {
            robot.setBulkReadToAuto();

            // movimentação padrão das partidas
            MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);

            // controles do braço e da mão
            ArmSystem.movimentarBraço(gamepad2, robot);

            //GripSystem.coletarCones(gamepad2, robot);

            // Debug de informações
            RobotLogger.debugControles(telemetry, gamepad1, gamepad2);
            //RobotLogger.showFulleMotorInfo(telemetry, robot.drivetrain.getMotor(RobotConstants.MOTOR_DIREITA_FRENTE_ID));
            //RobotLogger.showFulleMotorInfo(telemetry, robot.drivetrain.getMotor(RobotConstants.MOTOR_DIREITA_TRAS_ID));
            //RobotLogger.showFulleMotorInfo(telemetry, robot.drivetrain.getMotor(RobotConstants.MOTOR_ESQUERDA_FRENTE_ID));
            //RobotLogger.showFulleMotorInfo(telemetry, robot.drivetrain.getMotor(RobotConstants.MOTOR_ESQUERDA_TRAS_ID));
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.GripSystem;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

@TeleOp(name="TESTBOT", group = "Test")
public class FTC_TESTBOT extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // setup the hardware components
        DestemidosHardware robot = new DestemidosHardware(hardwareMap);
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
            RobotLogger.showFulleMotorInfo(telemetry, robot.motorDireitaFrente);
            RobotLogger.showFulleMotorInfo(telemetry, robot.motorDireitaTras);
            RobotLogger.showFulleMotorInfo(telemetry, robot.motorEsquerdaFrente);
            RobotLogger.showFulleMotorInfo(telemetry, robot.motorEsquerdaTras);
            telemetry.update();
        }
    }
}
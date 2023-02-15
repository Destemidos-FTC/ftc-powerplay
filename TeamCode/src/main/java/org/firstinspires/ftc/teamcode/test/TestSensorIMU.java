package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;

/*
 * TestSensorIMU - OpMode dedicado a leitura de informações
 * detalhadas como: Posição, Aceleração, Velocidade Angular, Orientação 
 * e etc.. todas providas pelo sensor embutido no próprio controlhub
 * 
 * para outros exemplos: https://stemrobotics.cs.pdx.edu/node/7265.html 
 */

@TeleOp(name = "TestSensorIMU", group = "Test")
@Disabled
public class TestSensorIMU extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DestemidosBot robot = new DestemidosBot(hardwareMap);
        MovementSystem movementSystem = new MovementSystem(robot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {
            movementSystem.standardMecanumController(gamepad1);

            // pegando informações do imu
            Orientation robotOrientation = robot.drivetrain.getSensorIMU().getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            );

            AngularVelocity robotAngVel = robot.drivetrain.getSensorIMU().getRobotAngularVelocity(AngleUnit.DEGREES);

            YawPitchRollAngles robotAngles = robot.drivetrain.getSensorIMU().getRobotYawPitchRollAngles();

            telemetry.addData("Sensor IMU - Velocidade Angular: ", robotAngVel);
            telemetry.addData("Sensor IMU - Orientação em Graus: ", "X: %d / Y: %d / Z: %d",
                    robotOrientation.firstAngle,
                    robotOrientation.secondAngle,
                    robotOrientation.thirdAngle
            );
            telemetry.addData("Sensor IMU - Ângulo das Rotações: ", "Yaw: %d / Pitch: %d / Row: %d",
                    robotAngles.getYaw(AngleUnit.DEGREES),
                    robotAngles.getPitch(AngleUnit.DEGREES),
                    robotAngles.getRoll(AngleUnit.DEGREES)
            );
        }
    }
}

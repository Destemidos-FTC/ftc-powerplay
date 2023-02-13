package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;
import java.util.List;

/*
 * (ramalho): segue abaixo o padrão de input pra qualquer função
 * que utilize os motores da base:
 *
 *      0 - Motor Direita-Frente
 *      1 - Motor Direita-Trás
 *      2 - Motor Esquerda-Frente
 *      3 - Motor Esquerda-Trás
 *
 * isso deve reforçar um comportamento padrão em todos os lugares, e reduz
 * algumas preocupações quando fomos debuggar
 *
 * referência: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 */
public final class Drivetrain {

    private final List<DcMotorEx> motors;
    public final DcMotorEx motorDireitaFrente;
    public final DcMotorEx motorDireitaTras;
    public final DcMotorEx motorEsquerdaFrente;
    public final DcMotorEx motorEsquerdaTras;
    private IMU sensorIMU;
    private IMU.Parameters ParametrosDoIMU;

    public Drivetrain(HardwareMap hardwareMap) {
        motorDireitaFrente = hardwareMap.get(DcMotorEx.class, "DF"); // porta 0 - controlHub
        motorDireitaTras = hardwareMap.get(DcMotorEx.class, "DT"); // porta 1 - controlHub
        motorEsquerdaFrente = hardwareMap.get(DcMotorEx.class, "EF"); // porta 2 - controlhub
        motorEsquerdaTras = hardwareMap.get(DcMotorEx.class, "ET"); // porta 3 - controlHub

        // precisamos inverter apenas os motores da esquerda
        motorEsquerdaFrente.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(motorDireitaFrente, motorDireitaTras, motorEsquerdaFrente, motorEsquerdaTras);

        resetEncoderWheels();

        configEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        configZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ParametrosDoIMU = new IMU.Parameters(
                // NOTE (ramalho): aqui é de acordo com a posição que colocamos o hub no robô
                // então é mais provável variar a direção das entradas USB nas futuras modificações do robô
                // referências: https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html

                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        // configurando e carregando o sensor IMU
        sensorIMU = hardwareMap.get(IMU.class, "imu");
        sensorIMU.initialize(ParametrosDoIMU);

    }

    public IMU getSensorIMU() {
        return sensorIMU;
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }

    public DcMotorEx getMotor(int ID) {
        return motors.get(ID);
    }

    public void setMotorsPower(double DireitaFrentePower, double DireitaTrasPower, double EsquerdaFrentePower, double EsquerdaTrasPower) {
        motorDireitaFrente.setPower(DireitaFrentePower);
        motorDireitaTras.setPower(DireitaTrasPower);
        motorEsquerdaFrente.setPower(EsquerdaFrentePower);
        motorEsquerdaTras.setPower(EsquerdaTrasPower);
    }

    public void setAllPower(double power) {
        motorDireitaFrente.setPower(power);
        motorDireitaTras.setPower(power);
        motorEsquerdaFrente.setPower(power);
        motorEsquerdaTras.setPower(power);
    }

    public void configEncoders(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void configZeroPowerBehavior(DcMotor.ZeroPowerBehavior ZeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(ZeroPowerBehavior);
        }
    }

    // Reinicia a contagem absoluta dos encoders
    public void resetEncoderWheels() {
        configEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveTank(Gamepad driver) {
        double joystick_y  = -driver.left_stick_y;
        double giro        = -driver.right_stick_x;

        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(giro), 1);

        double direitaFrentePower   = (joystick_y - giro) / denominador;
        double direitaTrasPower     = (joystick_y - giro) / denominador;
        double esquerdaFrentePower  = (joystick_y + giro) / denominador;
        double esquerdaTrasPower    = (joystick_y + giro) / denominador;

        setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
    }
    // Movimentação padrão e mais refinada do nosso controle, baseada no vídeo
    // extremamente didático do Gavin Ford: https://youtu.be/gnSW2QpkGXQ
    public void driveStandardMecanum(Gamepad driver)
    {
        double joystick_y  = driver.left_stick_y;  //* RobotConstants.MAX_SPEED;
        double joystick_x  = -driver.left_stick_x; // * RobotConstants.MAX_SPEED; //* RobotConstants.kCorretorJoystickX;
        double giro        = -driver.right_stick_x; //* RobotConstants.MAX_SPEED;

        // o denominador sempre será a maior força (valor absoluto) entre os 4 motores, ou equivalente a 1.
        // isso permite que todos mantenham a mesma taxa, mesmo que um motor ultrapasse os limites [-1, 1]
        //double denominador = Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro);

        //if(denominador > 1) {
        double direitaFrentePower   = (joystick_y - joystick_x - giro); // denominador;
        double direitaTrasPower     = (joystick_y + joystick_x - giro); // denominador;
        double esquerdaFrentePower  = (joystick_y + joystick_x + giro); // denominador;
        double esquerdaTrasPower    = (joystick_y - joystick_x + giro); // denominador;

        setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);

    }

    // Controle possivelmente mais preciso, baseado no método 2 do youtuber Gavin Ford
    public void drivePreciseMecanum(double theta, double direction, double turn) {

        double seno         = Math.sin(theta - RobotConstants.MECANUM_WHEELS_ANGLE);
        double cosseno      = Math.cos(theta - RobotConstants.MECANUM_WHEELS_ANGLE);

        // neste caso, o denominador vai "normalizar" os valores de cada eixo
        double denominador  = Math.max( Math.abs(seno), Math.abs(cosseno) );

        double direitaFrentePower   = (seno / denominador) - turn;
        double direitaTrasPower     = (cosseno / denominador) - turn;
        double esquerdaFrentePower  = (cosseno / denominador) + turn;
        double esquerdaTrasPower    = (seno / denominador) + turn;

        if(direction + Math.abs(turn) > 1) {
            direitaFrentePower  /= direction + Math.abs(turn);
            direitaTrasPower    /= direction + Math.abs(turn);
            esquerdaFrentePower /= direction + Math.abs(turn);
            esquerdaTrasPower   /= direction + Math.abs(turn);
        }
        
        setMotorsPower(
                direction * direitaFrentePower,
                direction * direitaTrasPower,
                direction * esquerdaFrentePower,
                direction * esquerdaTrasPower
        );
    }
    public void driveAlternativeMecanum(Gamepad driver) {
        double direction = Math.hypot(driver.left_stick_x, driver.left_stick_y);
        double robotAngle = Math.atan2(driver.left_stick_y, driver.left_stick_x) - Math.PI / 4;
        double rightX = driver.right_stick_x;

        final double v1 = direction * Math.cos(robotAngle) + rightX;
        final double v2 = direction * Math.sin(robotAngle) - rightX;
        final double v3 = direction * Math.sin(robotAngle) + rightX;
        final double v4 = direction * Math.cos(robotAngle) - rightX;

        setMotorsPower(v2, v4, v1, v3);
    }

    // um controle que sempre direciona o robô para onde apontamos no joystick, independente
    // da orentação do robô na arena
    public void driveFieldOriented(Gamepad driver) {
        double joystick_y  = -driver.left_stick_y  * RobotConstants.MAX_SPEED;
        double joystick_x  = -driver.left_stick_x  * RobotConstants.MAX_SPEED; //* RobotConstants.kCorretorJoystickX;
        double giro        = -driver.right_stick_x * RobotConstants.MAX_SPEED;

        double botHeading = getSensorIMU().getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;

        double rotationX = joystick_x * Math.cos(botHeading) - joystick_y * Math.sin(botHeading);
        double rotationY = joystick_x * Math.sin(botHeading) + joystick_y * Math.cos(botHeading);

        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro), 1);

        double direitaFrentePower   = (rotationY - rotationX - giro) / denominador;
        double direitaTrasPower     = (rotationY + rotationX - giro) / denominador;
        double esquerdaFrentePower  = (rotationY + rotationX + giro) / denominador;
        double esquerdaTrasPower    = (rotationY - rotationX + giro) / denominador;

        setMotorsPower(direitaFrentePower, direitaTrasPower, esquerdaFrentePower, esquerdaTrasPower);
    }
}
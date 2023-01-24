
@Config
public class Drivetrain {
    public static double MAX_SPEED = 1.0;

    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorDireitaTras;
    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorEsquerdaTras;
    private final List<DcMotorEx> motors;

    public enum Mode {
        MINIMAL,
        FULL
    }

    public Drivetrain(HardwareMap hardwareMap, ) {
        motorDireitaFrente  = hardwareMap.get(DcMotorEx.class,"DF"); // porta 0 - controlHub
        motorDireitaTras    = hardwareMap.get(DcMotorEx.class,"DT"); // porta 1 - controlHub
        motorEsquerdaFrente = hardwareMap.get(DcMotorEx.class,"EF"); // porta 3 - expansion
        motorEsquerdaTras   = hardwareMap.get(DcMotorEx.class,"ET"); // porta 3 - controlHub

        motors = Array.asList();
    }

    public void moveMotors() {
        
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

    // reinicia a contagem relativa dos encoders
    public void resetEncoderWheels(){
        configEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
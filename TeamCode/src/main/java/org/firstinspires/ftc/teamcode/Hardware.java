package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware {

    BNO055IMU imu;

    public DcMotor front_left = null;
    public DcMotor front_right = null;
    public DcMotor back_right = null;
    public DcMotor back_left = null;


    public DcMotor aspira1 = null;
    public DcMotor aspira2 = null;

    public DcMotorEx a1;
    public DcMotorEx a2;

    public DcMotorEx m;
    public DcMotorEx glis;

    public DcMotor rotatie = null;
    public DcMotor glisisus = null;

    public DigitalChannel atins1 = null;
    public DigitalChannel atins2 = null;

    public DistanceSensor distanta = null;
    public DistanceSensor placa;
    //public AnalogInput atins_glisi = null;

    public Servo placas, placad, brat,lasaCap,invarteCap,barieraCap; // de facut declaratii



    public CRServo cs, cd, bagas, bagad;

    public DcMotor getDcMotor(String name, HardwareMap hw )
    {
        if( hw.dcMotor.contains(name) )
            return hw.dcMotor.get(name);
        return null;
    }

    public Servo getServo(String name, HardwareMap hw )
    {
        if( hw.servo.contains(name) )
            return hw.servo.get(name);
        return null;
    }

    public CRServo getCRServo(String name, HardwareMap hw){
        if(hw.crservo.contains(name))
            return hw.crservo.get(name);
        return null;
    }

    public void init(HardwareMap hw){

        distanta = hw.get(DistanceSensor.class, "distanta");
        placa = hw.get(DistanceSensor.class, "placa");

        lasaCap = getServo("lasaCap", hw);
        invarteCap = getServo("invarteCap", hw);
        barieraCap = getServo("barieraCap", hw);
        barieraCap.setPosition(0.1);
        lasaCap.setPosition(0.7);
        invarteCap.setPosition(0);


        cd = getCRServo("cd", hw);
        cs = getCRServo("cs", hw);
        bagas = getCRServo("bagas", hw);
        bagad = getCRServo("bagad",hw);

        front_left = getDcMotor("front_left", hw);
        front_right = getDcMotor("front_right", hw);
        back_left = getDcMotor("back_left", hw);
        back_right = getDcMotor("back_right", hw);


        aspira1 = getDcMotor("aspira1", hw);
        aspira2 = getDcMotor("aspira2", hw);
        if(aspira1 != null && aspira2 != null) {
            aspira1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aspira1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aspira2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aspira2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        rotatie = getDcMotor("rotatie", hw);
        if(rotatie != null) {
            rotatie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotatie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotatie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        glisisus = getDcMotor("glisisus", hw);
        if(glisisus != null) {
            glisisus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            glisisus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            glisisus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        placad = getServo("placad", hw);
        placas = getServo("placas", hw);
        brat = getServo("brat", hw);
        if(brat!=null){
            brat.setPosition(0.2);
        }

        atins1 = hw.get(DigitalChannel.class, "atins1");
        atins1.setMode(DigitalChannel.Mode.INPUT);

        atins2 = hw.get(DigitalChannel.class, "atins2");
        atins2.setMode(DigitalChannel.Mode.INPUT);

        a1 = (DcMotorEx) aspira1;
        a2 = (DcMotorEx) aspira2;
        m = (DcMotorEx) front_left;
        glis = (DcMotorEx) glisisus;
        //atins_glisi = hw.get(AnalogInput.class, "atins_glisi");

        if( front_left != null )
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        if( back_left != null )
            back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        if( front_right != null )
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        if( back_right != null )
            back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        if(front_right != null && front_left != null && back_right != null && back_left != null) {
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            imu = hw.get(BNO055IMU.class, "imu");
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); // Retrieve and initialize the IMU

            imu.initialize(parameters);
        
    }

    public void stopRobot(){
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

}
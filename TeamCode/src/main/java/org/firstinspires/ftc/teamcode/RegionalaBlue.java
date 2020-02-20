package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vuforia.Position;
import org.firstinspires.ftc.teamcode.vuforia.PositionEnum;
import org.firstinspires.ftc.teamcode.vuforia.SkyStoneLocalizer;

import java.util.Objects;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;


@Autonomous(name = "Regionala Blue", group = "Linear OpMode")

public class RegionalaBlue extends LinearOpMode {

    Hardware robot = new Hardware();

    SkyStoneLocalizer localizer = new SkyStoneLocalizer();

    public double asta;
    double x,y;

    final double TICKSURI_PER_CM = 27.3696374577;

    private double inc1;
    private double inc2;
    private final double vit_max = 1;
    private final double lat_robot = 35;
    private final int ridicare = -450;
    ElapsedTime time = new ElapsedTime();

    public PositionEnum getEnumPosition(Position position) {
        Objects.requireNonNull(position);

        double x = position.getX();

        if (x < -150) {
            return PositionEnum.RIGHT;
        } else if (x >= -150 && x <= 150) {
            return PositionEnum.CENTER;
        } else {
            return PositionEnum.LEFT;
        }

    }


    private double distanta_placa = 30; // in mm

    public enum Direction{
        FRONT, BACK, LEFT, RIGHT;
    }

   /* public Position getPosition(){
        if(x < -70)
            return Position.RIGHT;
        else if(x > 140)
            return Position.LEFT;
        else
            return Position.MIDDLE;
    }

    public void setupVuforia(){
        cam = hardwareMap.get(WebcamName.class, "cam");

        param = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        param.vuforiaLicenseKey = KEY;

        param.cameraName = cam;
        Vuf = ClassFactory.createVuforiaLocalizer(param);

        targets = Vuf.loadTrackablesFromAsset("Skystone");
        allTrackables.addAll(targets);
    }

    public void tracking() {
        for(int i=0;i<5000;i++) {
            for (VuforiaTrackable trackable : allTrackables) {
                OpenGLMatrix pos = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (pos != null) {
                    lastLocation = pos;
                }
            }
            if (lastLocation != null) {
                seen = true;
                x = lastLocation.getTranslation().get(0);
                y = lastLocation.getTranslation().get(1);
//                telemetry.addData("x", x);
//                telemetry.addData("y", y);
//                telemetry.addData("Position", getPosition());
//                telemetry.update();
            }
        }
    }*/

   public void tracking(){
       for(int i=0;i<=1000;i++){
           Position position = localizer.getCurrentPosition();
           if(position.isValidPosition()) {
               x = position.getX();
               y = position.getX();
              /* telemetry.addLine(String.format("Target found (x=%s y=%s)", position.getX(), position.getY()));
               PositionEnum value = getEnumPosition(position);
               telemetry.addLine(String.format("Position: %s", value));*/
           } /*else {
               telemetry.addLine("No position!");
           }
           telemetry.update();*/
       }
   }

    double unghi(double u) {
        if (u < 0)
            u += 360;
        else if (u > 360)
            u -= 360;
        return u;
    }

    public void rotate(double speed){
        ///rotire in sens antitrigonometric in cazul puterii pozitive
        robot.front_left.setPower(speed);
        robot.front_right.setPower(-speed);
        robot.back_left.setPower(speed);
        robot.back_right.setPower(-speed);
    }

    public void Arc(Direction dir1, Direction dir2, double r, double angle, DcMotor glisisus, double ticks1, double pow1, DcMotor rotatie, double ticks2, double pow2) {
        if((dir1 == Direction.FRONT && dir2 == Direction.RIGHT)||( dir1 == Direction.BACK && dir2 == Direction.LEFT)) {
            angle = -angle;
        }
        double error=10;
        double current = unghi(robot.imu.getAngularOrientation().firstAngle);
        double target = unghi(current + angle);

        double remaining = Math.abs(angle);
        double ct = r / lat_robot;
        double in =-1.2* vit_max * r / (r + lat_robot);// ct * ex * r / (r + lat_robot)
        double ex = vit_max;

        double fl,fr,bl,br;
        if(dir2 == Direction.RIGHT){
            fl = bl = ex;
            fr = br = in;
        } else {
            fl = bl = in;
            fr = br = ex;
        }
        if(dir1 == Direction.BACK){
            fl = bl = -fl;
            fr = br = -fr;
        }

        double curent1=0, minpow1= -0.15;
        double curent2=0, minpow2=0, pr1 = pow2 + 0.2, pr2 = pow2 - 0.3;
        boolean glis = false;
        boolean rot = false;
        boolean stop = false;
        boolean brat = false;
        boolean tm = false;
        double poz = -5500;

        ElapsedTime time = new ElapsedTime();
        if(glisisus != null){
            curent1 = glisisus.getCurrentPosition();
            glis = true;
        }
        if(rotatie != null){
            curent2 = rotatie.getCurrentPosition();
            if(ticks2 <= 350 && ticks2 >= 40)
                minpow2 = 0.2;
            else if(ticks2 >= 710 && ticks2 <= 1300)
                minpow2 = -0.2;
        }


        while (opModeIsActive() && !isStopRequested() && remaining > error || glis || rot ) {
            current = unghi(robot.imu.getAngularOrientation().firstAngle);
            if (angle > 0) {
                if (target < current && current - target > 180) {
                    remaining = target - current + 360;
                } else {
                    remaining = target - current;
                }

            } else {
                if (target > current && target - current > 180) {
                    remaining = current - target + 360;
                } else {
                    remaining = current - target;
                }
            }

            if(remaining > error && !stop){
                robot.stopRobot();
                stop = true;
            }

            robot.front_left.setPower(fl);
            robot.back_left.setPower(0);
            robot.front_right.setPower(fr);
            robot.back_right.setPower(br);

            if(glisisus != null ){
                if(glis) {
                    if ((curent1 <= ticks1 && glisisus.getCurrentPosition() >= ticks1) || (curent1 >= ticks1 && glisisus.getCurrentPosition() <= ticks1)) {
                        glisisus.setPower(minpow1);
                        glis = false;
                    } else {
                        if (curent1 <= ticks1) {
                            glisisus.setPower(pow1);
                        } else {
                            glisisus.setPower(-pow1);
                        }
                    }
                }
                else if(!rot){
                    if(!brat) {
                        if(!tm){
                            time.reset();
                            tm = true;
                            robot.brat.setPosition(0.2);
                        }
                        else{
                            if(time.milliseconds() > 200){
                                brat = true;
                            }
                        }
                    }
                    else{
                       if(poz == -5500){
                           curent1 = glisisus.getCurrentPosition();
                           poz = Math.max(-2300, curent1 + ridicare);
                           pow1 = 0.8;
                       }
                       else{
                           if ((curent1 <= poz && glisisus.getCurrentPosition() >= poz) || (curent1 >= poz && glisisus.getCurrentPosition() <= poz)) {
                               glisisus.setPower(minpow1);
                               glis = false;
                               rot = true;
                           } else {
                               if (curent1 <= ticks1) {
                                   glisisus.setPower(pow1);
                               } else {
                                   glisisus.setPower(-pow1);
                               }
                           }
                       }
                    }
                }
            }

            if(rotatie != null && rot){
                if ((curent2 <=ticks2 && rotatie.getCurrentPosition() >= ticks2)||(curent2 >= ticks2 && rotatie.getCurrentPosition() <= ticks2)) {
                    rotatie.setPower(minpow2);
                    rot = false;
                } else {
                    if (curent2 <= ticks2) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                pow2 = pr2;
                            } else {
                                pow2 = pr1;
                            }
                            rotatie.setPower(pow2);
                    } else {
                            if (rotatie.getCurrentPosition() >= 550) {
                                pow2 = pr1;
                            }
                        rotatie.setPower(-pow2);
                    }
                }
            }


        }

        Redreseaza(target, false);
        robot.stopRobot();
    }

    public void Redreseaza(double target,boolean brate) {
        double MAXSPEED=0.25;
        double current = unghi(robot.imu.getAngularOrientation().firstAngle);
        double angle = target - current;

        double remaining = Math.abs(angle);

        double speed = MAXSPEED ;

        while (remaining > 1 && opModeIsActive() && !isStopRequested() ) {
          if(brate){
            if(!robot.atins1.getState())
                    robot.cs.setPower(0);
            if(!robot.atins2.getState())
                    robot.cd.setPower(0);}
            current = unghi(robot.imu.getAngularOrientation().firstAngle);

            if(Math.abs(robot.imu.getAngularVelocity().xRotationRate)<0.4)
                speed+=0.03;

            if (angle > 0) {
                if (target < current && current - target > 180) {
                    telemetry.addData("1", current);
                    remaining = target - current + 360;
                } else {
                    telemetry.addData("2", current);
                    remaining = target - current;
                }
                rotate(-speed);
            }
            else {
                if (target > current && target - current > 180) {
                    remaining = current - target + 360;

                } else {
                    remaining = current - target;
                }


                rotate(speed);
            }
            telemetry.addData("RRRremaining", remaining);
            telemetry.update();
        }
        robot.stopRobot();
    }


    public void gyroTurnV5(double angle, boolean brate, double power, double ticks, DcMotor motor) {
        double error = 2;
        double MAXSPEED = 1;
        double speed = MAXSPEED ;

        double curent=0, minpow=0,power1=0,power2=0;

        if(motor!=null)
        {
            curent = motor.getCurrentPosition();
            minpow = 0;

            power1 = power + 0.2;
            power2 = power - 0.3;
            if (motor == robot.rotatie) {
                if (ticks <= 350 && ticks >= 40)
                    minpow = 0.2;
                if (ticks >= 710 && ticks <= 1300)
                    minpow = -0.2;
            }

            if (motor == robot.glisisus)
                minpow = -0.15;
        }

        double current = unghi(robot.imu.getAngularOrientation().firstAngle);
        double target = unghi(current + angle);
        telemetry.addLine("current" + current);
        telemetry.addLine("target" + target);
        telemetry.update();

        double remaining = Math.abs(angle);
        double total = remaining / 2;
        if(angle<30)
            MAXSPEED=0.45;
        double viteza=100;

        while ((opModeIsActive() && !isStopRequested() )&& (remaining > error &&  viteza>0.07  )
                || (motor!=null && ((motor.getCurrentPosition() < ticks && curent <= ticks) ||(motor.getCurrentPosition() > ticks && curent >= ticks)))) {
            if(brate)
            {
                if(!robot.atins1.getState())
                    robot.cs.setPower(0);
                if(!robot.atins2.getState())
                    robot.cd.setPower(0);
                }

            if(motor!=null)
            {
                if ((curent <=ticks && motor.getCurrentPosition() >= ticks)||(curent >=ticks && motor.getCurrentPosition() <= ticks))
                {motor.setPower(minpow);
                }
                else {
                    if (curent <= ticks) {

                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power2;
                            } else {
                                power = power1;
                            }

//                            telemetry.addData("ticks In goToPosition", motor.getCurrentPosition());
//                            telemetry.addData("power In goToPosition", motor.getPower());
//                            telemetry.update();
                            motor.setPower(power);

                        }

                    } else {

                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power1;
                            }
                        }
                        motor.setPower(-power);
                    }
                }
             }

            if(remaining>total)
                viteza=100;
            else
                viteza=Math.abs(robot.imu.getAngularVelocity().xRotationRate);

            telemetry.addData("viteza", robot.imu.getAngularVelocity().xRotationRate);
            current = unghi(robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("target", target);
            if (angle > 0) {
                if (target < current && current - target > 180) {
                    telemetry.addData("1", current);
                    remaining = target - current + 360;
                } else {
                    telemetry.addData("2", current);
                    remaining = target - current;
                }
                if (remaining <= total) {
                    speed = MAXSPEED - ((total - remaining) * ((MAXSPEED - (0)) / total));
                }
                if(speed<0.35)
                    speed=0;
                rotate(-speed);
            } else {
                if (target > current && target - current > 180) {
                    remaining = current - target + 360;

                } else {
                    remaining = current - target;
                }
                if (remaining <= total) {
                    speed = MAXSPEED - ((total - remaining) * ((MAXSPEED - (0)) / total));
                }

                if(speed<0.35)
                    speed=0;
                rotate(speed);
            }
            telemetry.addData("glisisus", robot.glisisus.getCurrentPosition());
            telemetry.addData("remaining", remaining);
            telemetry.update();
        }
        asta=remaining;
        Redreseaza(target,brate);


        robot.stopRobot();
    }

    public void mersTicksuri(double distance, Direction dir, double pow, DcMotor motor, double ticks,
                             double power, boolean brate, boolean colectare, boolean tata){
      double curent=0, minpow=0,power1=0,power2=0;

        if(motor!=null)
        {
             curent = motor.getCurrentPosition();
             minpow = 0;

             power1 = power + 0.2;
             power2 = power - 0.3;
            if (motor == robot.rotatie) {
                if (ticks <= 350 && ticks >= 40)
                    minpow = 0.2;
                if (ticks >= 710 && ticks <= 1300)
                    minpow = -0.2;
            }

            if (motor == robot.glisisus)
                minpow = -0.15;
        }
        double fl,fr,br,bl;
        double speed = -1 * pow;
        fl = fr = br = bl = -1;
        if(dir == Direction.BACK)
            fl = fr = br = bl = 1;
        else if(dir == Direction.LEFT)
            fl = br = 1;
        else if(dir == Direction.RIGHT)
            fr = bl = 1;
        double initial = robot.back_left.getCurrentPosition();
        double necesar = distance * TICKSURI_PER_CM;
        double rem = 15 * TICKSURI_PER_CM;    /// decelerez pe ultimii 15 cm
        robot.front_left.setPower(speed * fl);
        robot.front_right.setPower(speed * fr);
        robot.back_right.setPower(speed * br);
        robot.back_left.setPower(speed * bl);
        boolean ver  = true;
        boolean b1 = false, b2= false;
        boolean mot = true;

        while((Math.abs(robot.back_left.getCurrentPosition() - initial)<necesar && (Math.abs(robot.m.getVelocity()) > 60 || ver))
               || (brate && (!b1 || !b2)) ||(motor!=null && mot)
                && !isStopRequested() && opModeIsActive() ){
            telemetry.addData("in MersTicks ", 1);
            telemetry.addData("b1", b1);
            telemetry.addData("B2", b2);
            telemetry.addData("rotatie", robot.rotatie.getCurrentPosition());
            telemetry.update();
            if(colectare)
            {
                if(robot.a1.getVelocity() > -100)
                    inc1 = -0.7;
                else
                    inc1 = 0;

                if(robot.a2.getVelocity() < 100)
                    inc2 = 0.7;
                else
                    inc2 = 0;

                robot.aspira1.setPower(-0.3 + inc1);
                robot.aspira2.setPower(0.3 + inc2);
                robot.bagas.setPower(-1);
                robot.bagad.setPower(1);

                telemetry.addData("a1 velocity", robot.a1.getVelocity());
                telemetry.addData("a2 velocity", robot.a2.getVelocity());
                telemetry.addData("a1 power", robot.aspira1.getPower());
                telemetry.addData("a2 power", robot.aspira2.getPower());
                telemetry.update();
            }
            if(brate){
                if (robot.atins1.getState()){
                    robot.cs.setPower(1);
                }
                else{
                    b1 = true;
                    robot.cs.setPower(0);
                }
                if (robot.atins2.getState())
                    robot.cd.setPower(-1);
                else{
                    b2 = true;
                    robot.cd.setPower(0);
                }
            }
            if(motor!=null) {

                if ((curent <=ticks && motor.getCurrentPosition() >= ticks)||(curent >=ticks && motor.getCurrentPosition() <= ticks)) {
                    motor.setPower(minpow);
                    mot = false;
                }
                else {
                    if (curent <= ticks) {
                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power2;
                            } else {
                                power = power1;
                            }
                        }
                         motor.setPower(power);

                    } else {
                        if (motor == robot.rotatie && robot.rotatie.getCurrentPosition() >= 550) {
                            power = power1;
                        }
                        motor.setPower(-power);
                    }
                }
            }
            double d = necesar - Math.abs(robot.back_left.getCurrentPosition() - initial);
            if(d < rem){
                ver=false;
                double v = -1 * pow * d / rem;
                if(v>-0.47 * pow)
                    v=0;
                robot.front_left.setPower(v * fl);
                robot.front_right.setPower(v * fr);
                robot.back_left.setPower(v * bl);
                robot.back_right.setPower(v * br);
            }
        }


        if(colectare) {
            while(robot.distanta.getDistance(MM)>60 && opModeIsActive() && !isStopRequested()){
                if(robot.a1.getVelocity() > -100)
                    inc1 = -0.7;
                else
                    inc1 = 0;

                if(robot.a2.getVelocity() < 100)
                    inc2 = 0.7;
                else
                    inc2 = 0;

                if(robot.distanta.getDistance(MM) <270){
                    inc1 = 1;
                    inc2 = -1;
                }
                robot.aspira1.setPower(-0.3 + inc1);
                robot.aspira2.setPower(0.3 + inc2);

                robot.bagas.setPower(-1);
                robot.bagad.setPower(1);
                telemetry.addData("a1 velocity", robot.a1.getVelocity());
                telemetry.addData("a2 velocity", robot.a2.getVelocity());
                telemetry.addData("a1 power", robot.aspira1.getPower());
                telemetry.addData("a2 power", robot.aspira2.getPower());
                telemetry.update();
            }

            robot.aspira1.setPower(0);
            robot.aspira2.setPower(0);
            robot.bagas.setPower(0);
            robot.bagad.setPower(0);
        }

        if(tata)
            robot.brat.setPosition(0.9);
    }


    private void Gliseaza(DcMotor glisisus, double ticks1, double pow1, DcMotor rotatie, double ticks2, double pow2){
        double curent1 = glisisus.getCurrentPosition();
        double curent2 = rotatie.getCurrentPosition();
        double minpow1 = -0.15;
        double minpow2 = 0;

        double power1 = pow2 + 0.2;
        double power2 = pow2 - 0.3;
        if (ticks2 <= 350 && ticks2 >= 40)
            minpow2 = 0.2;
        if(ticks2>=600 && ticks2<= 700)
            minpow2 = -0.05;
        if (ticks2 >= 710 && ticks2 <= 1300)
            minpow2 = -0.2;

        while( ((glisisus.getCurrentPosition() < ticks1 && curent1 <= ticks1) ||(glisisus.getCurrentPosition() > ticks1 && curent1 >= ticks1))
            || ((rotatie.getCurrentPosition() < ticks2 && curent2 <= ticks2) || (rotatie.getCurrentPosition() > ticks2 && curent2 >= ticks2) )
              && opModeIsActive() && !isStopRequested()) {

            if(((glisisus.getCurrentPosition() >= ticks1 && curent1 <= ticks1) ||(glisisus.getCurrentPosition() <= ticks1 && curent1 >= ticks1))) {
                glisisus.setPower(minpow1);
            }
            else {
                if (curent1 <= ticks1) {
                    glisisus.setPower(pow1);
                } else {
                    glisisus.setPower(-pow1);
                }
            }
            if((rotatie.getCurrentPosition() >= ticks2 && curent2 <= ticks2) || (rotatie.getCurrentPosition() <= ticks2 && curent2 >= ticks2) ){
                rotatie.setPower(minpow2);
            }
            else{
                if (curent2 <= ticks2) {

                    if (rotatie.getCurrentPosition() < ticks2) {
                        if (robot.rotatie.getCurrentPosition() >= 550) {
                            pow2 = power2;
                        } else {
                            pow2 = power1;
                        }
                        rotatie.setPower(pow2);
                    }
                } else {
                    if (rotatie.getCurrentPosition() > ticks2) {
                        if (robot.rotatie.getCurrentPosition() >= 550) {
                            pow2 = power1;
                        }
                        rotatie.setPower(-pow2);
                    }
                }
            }
        }
    }

    private void mersDistanta(double pow, DcMotor motor, double ticks, double power){
        double curent=0, minpow=0,power1=0,power2=0;

        if(motor!=null)
        {
            curent = motor.getCurrentPosition();
            minpow = 0;

            power1 = power + 0.2;
            power2 = power - 0.3;
            if (motor == robot.rotatie) {
                if (ticks <= 350 && ticks >= 40)
                    minpow = 0.2;
                if (ticks >= 710 && ticks <= 1300)
                    minpow = -0.2;
            }

            if (motor == robot.glisisus)
                minpow = -0.15;
        }
        double speed = -pow;
        double initial = robot.placa.getDistance(MM);
       // double necesar = initial - dist;
        double rem = initial/2;    /// decelerez pe ultimii 15 cm
        robot.front_left.setPower(speed);
        robot.front_right.setPower(speed);
        robot.back_right.setPower(speed);
        robot.back_left.setPower(speed);
        boolean ver = true;

        while((ver ||(motor!=null && ((motor.getCurrentPosition() < ticks && curent <= ticks) ||(motor.getCurrentPosition() > ticks && curent >= ticks))))
                && !isStopRequested() && opModeIsActive()){
            if(motor!=null) {

                if ((curent <=ticks && motor.getCurrentPosition() >= ticks)||(curent >=ticks && motor.getCurrentPosition() <= ticks))
                {motor.setPower(minpow);
                }
                else {
                    if (curent <= ticks) {

                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power2;
                            } else {
                                power = power1;
                            }

                            motor.setPower(power);
                        }
                    } else {
                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power1;
                            }
                        }
                        motor.setPower(-power);
                    }
                }

            }

            if(robot.placa.getDistance(MM) < 40 && ver){
                robot.stopRobot();
                ver = false;
            }
            telemetry.addData("distanta", robot.placa.getDistance(MM));
            telemetry.addData("viteza", robot.front_left.getPower());
            telemetry.update();

        }
        robot.stopRobot();

    }

    public void GoToPosition(DcMotor motor, int ticks, double power) {
        if(motor == null)
            return;
        double curent = motor.getCurrentPosition();
        double minpow = 0;

        double power1 = power + 0.2;
        double power2 = power - 0.3;
        if (motor == robot.rotatie) {
            if (ticks <= 350 && ticks >= 40)
                minpow = 0.2;
            if (ticks >= 710 && ticks <= 1300)
                minpow = -0.2;
        }

        if (motor == robot.glisisus)
            minpow = -0.15;

        if (curent <= ticks) {

            while (motor.getCurrentPosition() < ticks && opModeIsActive() && !isStopRequested()) {
                if (motor == robot.rotatie) {
                    if (robot.rotatie.getCurrentPosition() >= 550) {
                        power = power2;
                    } else {
                        power = power1;
                    }

                }
                telemetry.addData("ticks In goToPosition",motor.getCurrentPosition());
                telemetry.addData("power In goToPosition", motor.getPower());
                telemetry.update();
                motor.setPower(power);



            }
            motor.setPower(minpow);

        } else {

            while (motor.getCurrentPosition() > ticks && opModeIsActive() && !isStopRequested() ) {
                if (motor == robot.rotatie) {
                    if (robot.rotatie.getCurrentPosition() >= 550) {
                        power = power1;
                    }
                }
                motor.setPower(-power);
            }
            motor.setPower(minpow);
        }
    }

    private void Autonomie_Dreapta(){
        gyroTurnV5(-10-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);
        mersTicksuri(70, Direction.FRONT,0.3,null,0,0,false,true,false);
        mersTicksuri(45, Direction.BACK,1,robot.rotatie,30,0.6,false,false,true);

        gyroTurnV5(-90-robot.imu.getAngularOrientation().firstAngle, false,0,0,null);


        mersTicksuri(200, Direction.BACK,1,null,0,0,false,false,false);
        gyroTurnV5(-176-robot.imu.getAngularOrientation().firstAngle,false,1,-900,robot.glisisus);

        robot.placad.setPosition(0.35);  //0.35
        robot.placas.setPosition(0.2);  //0.2
        mersDistanta(0.2,robot.rotatie,1500,0.6);

        robot.placad.setPosition(0.5);  //0.35
        robot.placas.setPosition(0.45);  //0.2
        sleep(300);
        mersTicksuri(10, Direction.FRONT,0.6,null,0,0,false,false,false);
/***
 * baa
 * count ticks pt inainte sa FACEM ARCUL CA SA STI, CAT NE INTOARCEM CU PLACA
 */
        Arc(Direction.FRONT, Direction.LEFT, 10, 90, robot.glisisus, -600, 0.7, robot.rotatie, 50, 0.5);// de fact simutan motor

        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);
        Redreseaza(270, false);

        mersTicksuri(165 , Direction.FRONT,1,robot.glisisus,0,0.7,false,false,false);
        gyroTurnV5(-55-robot.imu.getAngularOrientation().firstAngle,false,0.4,200,robot.rotatie);
        // Redreseaza(300, false);
        mersTicksuri(45, Direction.FRONT, 0.3, null,0,0,false,true,false);//ia stone

        mersTicksuri(35, Direction.BACK,1,robot.rotatie,0,0.5,false,false,true); // se intoarce
        gyroTurnV5(-90-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);

        mersTicksuri(160, Direction.BACK,1,null,0,0,false,false,false);
        mersTicksuri(30, Direction.BACK,1,robot.glisisus,-1150,0.8,false,false,false);
        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);

        // robot.brat.setPosition(0.2);
        GoToPosition(robot.rotatie,1500,0.8);
        // Gliseaza(robot.glisisus,-1150,0.8, robot.rotatie,1500,0.8);
        mersTicksuri(30 , Direction.BACK,0.8,robot.glisisus,-850,0.6,false,false,false);
        /***
         * BAAAA
         * DE AFCUT SIMULTAN MERSUL DE #) CENTIMETRIIIIIII CU CELELALTE ACTIUNI(LA CARE PASTRAM ORDINEA)
         */
        robot.brat.setPosition(0.2);
        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);

        GoToPosition(robot.glisisus,-1200,1);
        GoToPosition(robot.rotatie, 50, 0.6);


        mersTicksuri(90, Direction.FRONT,1,robot.glisisus,0,0.6,false,false,false);

    }

    private void Autonomie_Mijloc(){
        gyroTurnV5(10,false,0,0,null);
        mersTicksuri(60, Direction.FRONT,0.2,null,0,0,false,true,false);
        mersTicksuri(28, Direction.BACK,1,robot.rotatie,30,0.5,false,false,true);

        gyroTurnV5(90-robot.imu.getAngularOrientation().firstAngle, false,0,0,null);


        mersTicksuri(190, Direction.FRONT,1,null,0,0,false,false,false);
        gyroTurnV5(180-robot.imu.getAngularOrientation().firstAngle,false,1,-900,robot.glisisus);

        mersDistanta(0.2,robot.rotatie,1500,0.6);

        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);
        sleep(300);
        mersTicksuri(10, Direction.FRONT,0.6,null,0,0,false,false,false);

         Arc(Direction.FRONT, Direction.LEFT, 10, 90, robot.glisisus, -600, 0.7, robot.rotatie, 50, 0.7);// de fact simutan motor

        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);
        Redreseaza(270, false);

        mersTicksuri(155 , Direction.FRONT,1,robot.glisisus,0,0.7,false,false,false);
        gyroTurnV5(-55-robot.imu.getAngularOrientation().firstAngle,false,0.6,200,robot.rotatie);
       // Redreseaza(300, false);
        mersTicksuri(40, Direction.FRONT, 0.3, null,0,0,false,true,false);//ia stone

        mersTicksuri(35, Direction.BACK,1,robot.rotatie,0,0.5,false,false,true); // se intoarce
        gyroTurnV5(-90-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);

        mersTicksuri(150, Direction.BACK,1,null,0,0,false,false,false);
        mersTicksuri(30, Direction.BACK,1,robot.glisisus,-1150,0.8,false,false,false);
        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);

       // robot.brat.setPosition(0.2);
        GoToPosition(robot.rotatie,1500,0.8);
       // Gliseaza(robot.glisisus,-1150,0.8, robot.rotatie,1500,0.8);
        mersTicksuri(30 , Direction.BACK,0.8,robot.glisisus,-850,0.6,false,false,false);
        /***
         * BAAAA
         * DE AFCUT SIMULTAN MERSUL DE #) CENTIMETRIIIIIII CU CELELALTE ACTIUNI(LA CARE PASTRAM ORDINEA)
         */
        robot.brat.setPosition(0.2);
        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);

        GoToPosition(robot.glisisus,-1200,1);
        GoToPosition(robot.rotatie, 50, 0.6);


        mersTicksuri(90, Direction.FRONT,1,robot.glisisus,0,0.6,false,false,false);

    }

    private void Autonomie_Stanga(){

        gyroTurnV5(25-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);
        mersTicksuri(70, Direction.FRONT,0.3,null,0,0,false,true,false);
        mersTicksuri(43, Direction.BACK,1,robot.rotatie,30,0.6,false,false,true);

        gyroTurnV5(90-robot.imu.getAngularOrientation().firstAngle, false,0,0,null);


        mersTicksuri(190, Direction.FRONT,1,null,0,0,false,false,false);
        gyroTurnV5(180-robot.imu.getAngularOrientation().firstAngle,false,1,-900,robot.glisisus);

        robot.placad.setPosition(0.35);  //0.35
        robot.placas.setPosition(0.2);  //0.2
        mersDistanta(0.2,robot.rotatie,1500,0.6);

        robot.placad.setPosition(0.5);  //0.35
        robot.placas.setPosition(0.45);  //0.2
        sleep(300);
        mersTicksuri(10, Direction.FRONT,0.6,null,0,0,false,false,false);
/***
 * baa
 * count ticks pt inainte sa FACEM ARCUL CA SA STI, CAT NE INTOARCEM CU PLACA
 */
        Arc(Direction.FRONT, Direction.LEFT, 10, 90, robot.glisisus, -600, 0.7, robot.rotatie, 50, 0.7);// de fact simutan motor

        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);
        Redreseaza(270, false);

        mersTicksuri(145 , Direction.FRONT,1,robot.glisisus,0,0.7,false,false,false);
        gyroTurnV5(-55-robot.imu.getAngularOrientation().firstAngle,false,0.4,200,robot.rotatie);
        // Redreseaza(300, false);
        mersTicksuri(45, Direction.FRONT, 0.3, null,0,0,false,true,false);//ia stone

        mersTicksuri(40, Direction.BACK,1,robot.rotatie,0,0.5,false,false,true); // se intoarce
        gyroTurnV5(-90-robot.imu.getAngularOrientation().firstAngle,false,0,0,null);

        mersTicksuri(140, Direction.BACK,1,null,0,0,false,false,false);
        mersTicksuri(20, Direction.BACK,1,robot.glisisus,-1150,0.8,false,false,false);
        robot.placad.setPosition(0.5);
        robot.placas.setPosition(0.45);

        // robot.brat.setPosition(0.2);
        //GoToPosition(robot.rotatie,1500,0.8);
        // Gliseaza(robot.glisisus,-1150,0.8, robot.rotatie,1500,0.8);

        mersTicksuri(20, Direction.BACK,1,robot.rotatie,1500,0.8,false,false,false);
        mersTicksuri(20 , Direction.BACK,1,robot.glisisus,-850,0.6,false,false,false);
        /***
         * BAAAA
         * DE AFCUT SIMULTAN MERSUL DE #) CENTIMETRIIIIIII CU CELELALTE ACTIUNI(LA CARE PASTRAM ORDINEA)
         */
        robot.brat.setPosition(0.2);
        robot.placad.setPosition(0.2);
        robot.placas.setPosition(0);


        GoToPosition(robot.glisisus,-1200,1);
        GoToPosition(robot.rotatie, 50, 0.6);


        mersTicksuri(90, Direction.FRONT,1,robot.glisisus,0,0.6,false,false,false);


    }

    private void Laterala(Direction dir, double pow, double distance){
        double fl = 1,bl = 1,fr = 1,br = 1;
        if(dir == Direction.RIGHT)
            fr = bl = -1;
        else
            fl = br = -1;
        double initial = robot.back_left.getCurrentPosition();
        double necesar = distance * TICKSURI_PER_CM;
        double rem = 15 * TICKSURI_PER_CM;    /// decelerez pe ultimii 15 cm
        double st = 15 * TICKSURI_PER_CM;
        boolean ver = true;
        while(Math.abs(robot.back_left.getCurrentPosition() - initial) < necesar && opModeIsActive() && !isStopRequested()){
            telemetry.addData("front left", robot.front_left.getPower());
            telemetry.addData("back left", robot.back_left.getPower());
            telemetry.addData("front right", robot.front_right.getPower());
            telemetry.addData("back right", robot.back_right.getPower());
            telemetry.update();
            double done = Math.abs(robot.back_left.getCurrentPosition() - initial);
            double d = necesar - done;  // distanta ramasa
            if(done < st){
                double v = 0.4 + 0.6 * done / st;
                robot.front_left.setPower(v * fl);
                robot.front_right.setPower(v * fr);
                robot.back_left.setPower(v * bl);
                robot.back_right.setPower(v * br);
            }
          /*  else if(d < rem){
                ver=false;
                double v =  pow * d / rem;
                if(v < 0.47 * pow)
                    v=0;
                robot.front_left.setPower(v * fl);
                robot.front_right.setPower(v * fr);
                robot.back_left.setPower(v * bl);
                robot.back_right.setPower(v * br);
            }*/
        }
        robot.stopRobot();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
       robot.init(hardwareMap);
      // localizer.init(hardwareMap);
       telemetry.addLine("Robot is initialised.");
       telemetry.update();

       waitForStart();
       mersTicksuri(35, Direction.FRONT,1, robot.rotatie, 200, 0.4, true, false, false);
//       localizer.activate();
//       tracking();
//       telemetry.addData("X:", x);
//       telemetry.addData("Y:", y);
//       telemetry.update();


      /* if(!seen){
           Autonomie_Stanga();
       }
       else {
           if (getPosition() == Position.LEFT)
               Autonomie_Stanga();
           else if(getPosition() == Position.MIDDLE)
               Autonomie_Mijloc();
           else
               Autonomie_Dreapta();
       }*/
     // Autonomie_Dreapta();
    }
}
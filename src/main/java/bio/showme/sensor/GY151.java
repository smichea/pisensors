package bio.showme.sensor;

import io.dvlopt.linux.i2c.* ;

import java.io.IOException;
public class GY151 extends Sensor {

    private static I2CBus bus = null;
    private static I2CBus gy151 = null; //this is what the bus becomes when a slave is selected, it correspond to pi4j device

    private static final int Device_Address=0x68;
    private static final int PWR_MGMT_1    =0x6B;
    private static final int SMPLRT_DIV    =0x19;
    private static final int CONFIG        =0x1A;
    private static final int GYRO_CONFIG   =0x1B;
    private static final int ACCEL_CONFIG  =0x1C;
    private static final int INT_ENABLE    =0x38;
    private static final byte[] ACC_REGISTERS = {0x3B,0x3D,0x3F};
    private static final String[] ACC_REGISTER_NAMES = {"Acc_x","Acc_y","Acc_z"};
    private static final byte[] GYRO_REGISTERS = {0x43,0x45,0x47};
    private static final String[] GYRO_REGISTER_NAMES = {"Gy_x","Gy_y","Gy_z"};
    public static final int BUFFER_SIZE = 100;

    public static final double RAD2DEG  = 57.29577951308; // 180/pi

    private static float[] acc_precision = new float[ACC_REGISTERS.length];
    private static float[] acc_initiale = new float[ACC_REGISTERS.length];
    private static int[][] accBufferValues = new int[ACC_REGISTERS.length][BUFFER_SIZE];
    private static float[] gyro_delta = new float[GYRO_REGISTERS.length];
    private static int[][] gyroBufferValues = new int[GYRO_REGISTERS.length][BUFFER_SIZE];
    long lastTimestamp=0;
    double[] quaternionAngles = new double[4];

    private static final int GYRO_RANGE = 0;
    private static double GYRO_SENS_VAL = 131.0;
    private static byte GYRO_CONFIG_VAL = 0b00000000;
    static {
        if(GYRO_RANGE == 1) {
            GYRO_SENS_VAL=65.5;
            GYRO_CONFIG_VAL=0b00001000;
        } else if (GYRO_RANGE == 2) {
            GYRO_SENS_VAL=32.8;
            GYRO_CONFIG_VAL=0b00010000;
        } else if (GYRO_RANGE == 3) {
            GYRO_SENS_VAL=16.4;
            GYRO_CONFIG_VAL=0b00011000;
        }
    }

    private static final int ACC_RANGE = 0;
    private static double ACC_SENS_VAL = 16384.0;
    private static byte ACC_CONFIG_VAL = 0b00000000;

    static {
        if(ACC_RANGE == 1) {
            ACC_SENS_VAL= 8192.0;
            ACC_CONFIG_VAL=0b00001000;
        } else if (ACC_RANGE == 2) {
            ACC_SENS_VAL= 4096.0;
            ACC_CONFIG_VAL=0b00010000;
        } else if (ACC_RANGE == 3) {
            ACC_SENS_VAL= 2048.0;
            ACC_CONFIG_VAL=0b00011000;
        }
    }

    public GY151(String initConfig) throws Exception {
        super();
        init(initConfig);
    }

    void init(String config) throws Exception {
        sensorId=151;
        registerNames=  new String[]{"Acc_x","Acc_y","Acc_z","Gy_x","Gy_y","Gy_z"};
        bus = new I2CBus( 1) ;//I2CFactory.getInstance(I2CBus.BUS_1);
        bus.selectSlave( Device_Address ) ;//bus.getDevice(Device_Address);
        gy151 = bus;//bus.getDevice(Device_Address);
        //writeConfigRegister("Waking up device", PWR_MGMT_1, (byte)0x01);
        //writeConfigRegister("Configuring sample rate", SMPLRT_DIV, (byte)0x07);
        //writeConfigRegister("Setting global config (digital low pass filter)",CONFIG, (byte)0);
        //writeConfigRegister("Configuring gyroscope", GYRO_CONFIG, (byte)24);
        //writeConfigRegister("Configuring accelerometer", ACCEL_CONFIG, (byte)0);
        //writeConfigRegister("Configuring interrupts", INT_ENABLE, (byte)1);
        writeConfigRegister("Waking up device", PWR_MGMT_1, (byte)0);
        writeConfigRegister("Setting global config (digital low pass filter)",CONFIG, (byte)0b00000011);
        writeConfigRegister("Configuring sample rate divider (to 200Hz)", SMPLRT_DIV, (byte)0b00000100);
        writeConfigRegister("Configuring gyroscope", GYRO_CONFIG, GYRO_CONFIG_VAL);
        writeConfigRegister("Configuring accelerometer", ACCEL_CONFIG, ACC_CONFIG_VAL);
        writeConfigRegister("Configuring interrupts", INT_ENABLE, (byte)1);
        writeConfigRegister("", 0x06, (byte)0b00000000);
        writeConfigRegister("", 0x07, (byte)0b00000000);
        writeConfigRegister("", 0x08, (byte)0b00000000);
        writeConfigRegister("", 0x09, (byte)0b00000000);
        writeConfigRegister("", 0x0A, (byte)0b00000000);
        writeConfigRegister("", 0x0B, (byte)0b00000000);
        writeConfigRegister("", 0x00, (byte)0b10000001);
        writeConfigRegister("", 0x01, (byte)0b00000001);
        writeConfigRegister("", 0x02, (byte)0b10000001);


        writeConfigRegister("", 0x06, (byte)0b00000000);
        writeConfigRegister("", 0x07, (byte)0b00000000);
        writeConfigRegister("", 0x08, (byte)0b00000000);
        writeConfigRegister("", 0x09, (byte)0b00000000);
        writeConfigRegister("", 0x0A, (byte)0b00000000);
        writeConfigRegister("", 0x0B, (byte)0b00000000);
        writeConfigRegister("", 0x00, (byte)0b10000001);
        writeConfigRegister("", 0x01, (byte)0b00000001);
        writeConfigRegister("", 0x02, (byte)0b10000001);

        initiated=true;
    }

    private static void writeRegister(int register, byte data) throws IOException {
        I2CBuffer buff = new I2CBuffer(2).set(0,(byte)register).set(1,data);
        gy151.write(buff);
    }

    public static byte readRegister(int register) throws IOException {
        I2CBuffer buff = new I2CBuffer(1).set(0,(byte)register);
        gy151.write(buff,1);
        gy151.read(buff);
        int data = buff.get(0);
        return (byte) data;
    }

    public static int readIntRegister(int register) throws IOException {
        I2CBuffer buff = new I2CBuffer(2);
        gy151.read(buff);
        byte h = (byte)buff.get(0);
        //System.out.print("\n h="+h);
        byte l= (byte)buff.get(1);
        //System.out.print("\n l="+l);
        int res = h*256+l;
        //System.out.print(" h<<8+l="+res);
        return res;
    }

    public static void writeConfigRegister(String initialText, int register, byte registerData) throws IOException {
        System.out.println(initialText);
        writeRegister(register, registerData);
        byte returnedRegisterData = readRegister(register);
        if (returnedRegisterData == registerData) {
            System.out.println(" success " + formatBinary(returnedRegisterData));
        } else {
            throw new RuntimeException("Tried to write " + formatBinary(registerData) + " to "
                    + register + ", but validating value returned " + formatBinary(returnedRegisterData));
        }
    }

    public static String formatBinary(byte b) {
        String binaryString = Integer.toBinaryString(b);
        if (binaryString.length() > 8) {
            binaryString = binaryString.substring(binaryString.length() - 8);
        }
        if (binaryString.length() < 8) {
            byte fillingZeros = (byte) (8 - binaryString.length());
            for (int j = 1; j <= fillingZeros; j++) {
                binaryString = "0" + binaryString;
            }
        }
        return binaryString;
    }

    public double[] getRawAcc(){
        double[] result= new double[3];
        for(int j=0;j<ACC_REGISTERS.length;j++){
            try {
                result[j] = 0.0 + readIntRegister(ACC_REGISTERS[j]);
            } catch (Exception e ){}
        }
        return result;
    }

    public double[] getAcc(){
        double[] result=getRawAcc();
        for(int j=0;j<ACC_REGISTERS.length;j++){
            result[j] = Math.round((result[j] - acc_initiale[j]) * 1000.0 / ACC_SENS_VAL) / 1000.0;
        }
        return result;
    }

    public static double[] radianEulerToQuaternions(double rollRad,double pitchRad,double yawRad){
        double[] result =  new double[4];
        result[0] = Math.sin(rollRad/2)*Math.cos(pitchRad/2)*Math.cos(yawRad/2) - Math.cos(rollRad/2)*Math.sin(pitchRad/2)*Math.sin(yawRad/2) ;
        result[1] = Math.cos(rollRad/2)*Math.sin(pitchRad/2)*Math.cos(yawRad/2) + Math.sin(rollRad/2)*Math.cos(pitchRad/2)*Math.sin(yawRad/2) ;
        result[3] = Math.cos(rollRad/2)*Math.cos(pitchRad/2)*Math.sin(yawRad/2) - Math.sin(rollRad/2)*Math.sin(pitchRad/2)*Math.cos(yawRad/2) ;
	    result[4] = Math.cos(rollRad/2)*Math.cos(pitchRad/2)*Math.cos(yawRad/2) + Math.sin(rollRad/2)*Math.sin(pitchRad/2)*Math.sin(yawRad/2) ;
        return result;
    }

    public static double[] quaternionsToRadianEuler(double Q1,double Q2,double Q3,double Q4){
        double[] result=new double[3];
        result[0] = Math.atan2( 2*( Q4*Q1 + Q2*Q3 ) , 1 - 2*( Q1*Q1 + Q2*Q2 ) ) ;
        result[1] = Math.asin( 2*( Q4*Q2 - Q3*Q1 ) ) ;
        result[2] = Math.atan2( 2*( Q4*Q3 + Q1*Q2 ) , 1 - 2*( Q2*Q2 + Q3*Q3) ) ;
        return result;
    }
    public static double[] quaternionIncrement(double rateXDeg,double rateYDeg,double rateZDeg,double[] q){
        double[] result = new double[4];
        double rateXRad = rateXDeg / RAD2DEG ;
        double rateYRad = rateYDeg / RAD2DEG ;
        double rateZRad = rateZDeg / RAD2DEG ;
        result[0] = 0.5 * ( 0 + rateZRad*q[1] - rateYRad*q[2] + rateXRad*q[3] ) ;
        result[1] = 0.5 * ( 0 - rateZRad*q[0] + rateXRad*q[2] + rateYRad*q[3] ) ;
        result[2] = 0.5 * ( 0 + rateYRad*q[0] - rateXRad*q[1] + rateZRad*q[3] ) ;
        result[3] = 0.5 * ( 0 - rateXRad*q[0] - rateYRad*q[1] - rateZRad*q[2] ) ;
        return result;
    }


    public static void integrate(double[] quaternionAngles,double[] quaternionIncrement,double timeDelta){
        for(int i=0;i<4;i++) {
            quaternionAngles[i] = quaternionAngles[0] + timeDelta * quaternionIncrement[i];
        }
        double norm = Math.sqrt(quaternionAngles[0]*quaternionAngles[0]+quaternionAngles[1]*quaternionAngles[1]
                +quaternionAngles[2]*quaternionAngles[2]+quaternionAngles[3]*quaternionAngles[3]);
        for(int i=0;i<4;i++) {
            quaternionAngles[i] /= norm;
        }
    }


    public void calibrate(String config){
        for(int j=0;j<GYRO_REGISTERS.length;j++){
            gyro_delta[j]=0.0f;
        }
        for(int j=0;j<ACC_REGISTERS.length;j++){
            acc_precision[j]=0.0f;
        }
        for(int i=0;i<BUFFER_SIZE;i++){
            for(int j=0;j<GYRO_REGISTERS.length;j++){
                try {
                    gyroBufferValues[j][i] = readIntRegister(GYRO_REGISTERS[j]);
                } catch (IOException e){
                    if(i>0) {
                        gyroBufferValues[j][i] = gyroBufferValues[j][i-1];
                    } else {
                        gyroBufferValues[j][i] = 0;
                    }
                }
                gyro_delta[j]+= gyroBufferValues[j][i];
            }
            for(int j=0;j<ACC_REGISTERS.length;j++){
                try {
                    accBufferValues[j][i] = readIntRegister(ACC_REGISTERS[j]);
                } catch (IOException e){
                    if(i>0) {
                        accBufferValues[j][i] = accBufferValues[j][i-1];
                    } else {
                        accBufferValues[j][i] = 0;
                    }
                }
            }
        }

        for(int j=0;j<ACC_REGISTERS.length;j++){
            float min= accBufferValues[j][0];
            float max= accBufferValues[j][0];
            for(int i=0;i<BUFFER_SIZE;i++){
                if(accBufferValues[j][i]<min){
                    min=accBufferValues[j][i];
                } else if(accBufferValues[j][i]>max){
                    max=accBufferValues[j][i];
                }
            }
            acc_precision[j]=max-min;
            acc_initiale[j]=min+(acc_precision[j]/2);
        }

        System.out.print("\n");
        for(int j=0;j<ACC_REGISTERS.length;j++){
            acc_precision[j]/=BUFFER_SIZE+0.0f;
            System.out.print(ACC_REGISTER_NAMES[j]+" precision="+ acc_precision[j]);
        }
        for(int j=0;j<GYRO_REGISTERS.length;j++){
            gyro_delta[j]/=BUFFER_SIZE+0.0f;
            System.out.print(GYRO_REGISTER_NAMES[j]+" delta="+ gyro_delta[j]);
        }
        double initRoll = 0.0;
        double initPitch = 0.0;
        lastTimestamp=System.currentTimeMillis();
        double[] acc = getAcc();
        for (int i=1;i<BUFFER_SIZE;i++){
            initRoll -= Math.atan2(-1.0*acc[1],acc[2]);
            initPitch -= Math.asin(acc[0]) ;
        }
        initRoll = Math.round(initRoll * 1000 / BUFFER_SIZE ) / 1000 ;
        initPitch = Math.round(initPitch * 1000 / BUFFER_SIZE ) / 1000 ;

        quaternionAngles = radianEulerToQuaternions(initRoll, initPitch,0.0);

    }



    int[] rawValues = new int[GYRO_REGISTERS.length];
    void measure(int[] values){
        try {
                //read raw acc
                for(int j=0;j<ACC_REGISTERS.length;j++){
                    values[j]=readIntRegister(ACC_REGISTERS[j]);
                }
                //read raw gyro
                for(int j=0;j<GYRO_REGISTERS.length;j++){
                    rawValues[j]=readIntRegister(GYRO_REGISTERS[j]);
                }
                //adjust with bias... not sure this is meaningful for acceleration
                //for(int j=0;j<ACC_REGISTERS.length;j++) {
                //    values[j] = (int) Math.round((rawValues[j] - gyro_delta[j]) * 1000.0 / ACC_SENS_VAL) / 1000;
                //}
                double[] gyro=new double[3];
                for(int j=0;j<3;j++) {
                    gyro[j] = Math.round((rawValues[j] - gyro_delta[j]) * 1000.0 / GYRO_SENS_VAL) / 1000.0;
                }
                double[] quaternionIncrement = quaternionIncrement(gyro[0],gyro[1],gyro[2],quaternionAngles);
                long timeStamp = System.currentTimeMillis();
                integrate(quaternionAngles,quaternionIncrement,(timeStamp-lastTimestamp)/1000.0);
                lastTimestamp=timeStamp;
                double[] eulerAngles = quaternionsToRadianEuler(quaternionAngles[0],quaternionAngles[1],quaternionAngles[2],quaternionAngles[3]);
                for(int j=0;j<3;j++){
                    values[ACC_REGISTERS.length+j]=(int)Math.round(eulerAngles[j]*RAD2DEG);
                }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

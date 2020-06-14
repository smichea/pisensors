package bio.showme.sensor;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;

import java.io.IOException;
public class GY151 extends Sensor {

    private static I2CBus bus = null;
    private static I2CDevice gy151 = null;

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

    private static float[] acc_precision = new float[ACC_REGISTERS.length];
    private static float[] acc_initiale = new float[ACC_REGISTERS.length];
    private static int[][] accBufferValues = new int[ACC_REGISTERS.length][BUFFER_SIZE];
    private static float[] gyro_delta = new float[GYRO_REGISTERS.length];
    private static int[][] gyroBufferValues = new int[GYRO_REGISTERS.length][BUFFER_SIZE];

    public GY151(String initConfig) throws Exception {
        super();
        init(initConfig);
    }

    void init(String config) throws Exception {
        sensorId=151;
        registerNames=  new String[]{"Acc_x","Acc_y","Acc_z","Gy_x","Gy_y","Gy_z"};
        bus = I2CFactory.getInstance(I2CBus.BUS_1);
        gy151 = bus.getDevice(Device_Address);
        writeConfigRegister("Waking up device", PWR_MGMT_1, (byte)0x01);
        writeConfigRegister("Configuring sample rate", SMPLRT_DIV, (byte)0x07);
        writeConfigRegister("Setting global config (digital low pass filter)",CONFIG, (byte)0);
        writeConfigRegister("Configuring gyroscope", GYRO_CONFIG, (byte)24);
        writeConfigRegister("Configuring accelerometer", ACCEL_CONFIG, (byte)0);
        writeConfigRegister("Configuring interrupts", INT_ENABLE, (byte)1);
        initiated=true;
    }

    private static void writeRegister(int register, byte data) throws IOException {
        gy151.write(register, data);
    }

    public static byte readRegister(int register) throws IOException {
        int data = gy151.read(register);
        return (byte) data;
    }

    public static int readIntRegister(int register) throws IOException {
        byte h = (byte)gy151.read(register);
        //System.out.print("\n h="+h);
        byte l= (byte)gy151.read(register+1);
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
            acc_initiale[j]=min;
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
    }

    void measure(int[] values){
        try {
                for(int j=0;j<ACC_REGISTERS.length;j++){
                    values[j]=readIntRegister(ACC_REGISTERS[j]);
                }
                for(int j=0;j<GYRO_REGISTERS.length;j++){
                    values[j+ACC_REGISTERS.length]=readIntRegister(GYRO_REGISTERS[j]);
                }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

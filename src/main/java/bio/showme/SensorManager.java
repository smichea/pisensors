package bio.showme;

import bio.showme.sensor.GY151;
import bio.showme.sensor.Mouse;
import bio.showme.sensor.Sensor;
import bio.showme.server.SensorsServer;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class SensorManager {

    List<Sensor> sensors = new ArrayList<>();
    List<MeasureListener> measureListeners = new ArrayList<>();

    private SensorManager(){}

    public void registerSensor(Sensor sensor){
        if(!sensors.contains(sensor)){
            sensors.add(sensor);
        }
        if(measureListeners.size()>0){
            for(MeasureListener measureListener:measureListeners){
                sensor.registerMeasureListener(measureListener);
            }
        }
    }

    public void registerListener(MeasureListener measureListener){
        if(!measureListeners.contains(measureListener)){
            measureListeners.add(measureListener);
        }
        if(sensors.size()>0){
            for(Sensor sensor:sensors){
                sensor.registerMeasureListener(measureListener);
            }
        }
    }

    public static void main(String[] args){
        SensorManager sensorManager=new SensorManager();
        try {
            Mouse mouse = new Mouse("");
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            Sensor gyro = new GY151("");
            gyro.calibrate("");
            sensorManager.registerSensor(gyro);
            Thread device1 =new Thread(gyro);
            device1.start();
        } catch(Exception e){
            e.printStackTrace();
        }
        SensorsServer s=null;
        try {
            int port = 6340;
            s = new SensorsServer( port );
            s.start();
            sensorManager.registerListener(s);
            System.out.println( "SensorsServer started on port: " + s.getPort() );
            BufferedReader sysin = new BufferedReader( new InputStreamReader( System.in ) );
            while ( true ) {
                String in = sysin.readLine();
                s.broadcast( in );
                if( in.equals( "exit" ) ) {
                    s.stop(1000);
                    break;
                }
            }
        } catch ( Exception ex ) {
            ex.printStackTrace();
            if(s!=null){
                try {
                    s.stop(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}

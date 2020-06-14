package bio.showme.sensor;

import bio.showme.MeasureListener;

import java.util.ArrayList;
import java.util.List;

public abstract class Sensor implements Runnable {

    int sensorId = 0;
    boolean initiated = false;
    boolean stopDeviceMeasures;
    public String[] registerNames;
    public int measurePollDelay = 100;

    private List<MeasureListener> measureListeners = new ArrayList<>();

    abstract void init(String config) throws Exception;
    abstract void measure(int[] values) throws Exception;

    public abstract void calibrate(String config);

    public void registerMeasureListener(MeasureListener measureListener){
        if(!measureListeners.contains(measureListener)){
            measureListeners.add(measureListener);
        }
    }

    public void removeMeasureListener(MeasureListener measureListener){
        if(measureListeners.contains(measureListener)){
            measureListeners.remove(measureListener);
        }
    }

    public void sendMeasure(long timestamp,int measureDelayMs,int[] measure){
        if(measureListeners.size()>0){
            for (MeasureListener measureListener:measureListeners){
                measureListener.measured(timestamp,measureDelayMs,sensorId,measure);
            }
        } else {
            System.out.print("\n "+timestamp+"["+measureDelayMs+"],#"+sensorId);
            for(int i=0;i<registerNames.length;i++){
                System.out.print(", "+registerNames[i]+"="+measure[i]);
            }
        }
    }

    Sensor(){
    }

    @Override
    public void run() {
        int[] values = new int[registerNames.length];
        try {
            while (!stopDeviceMeasures) {
                try {
                    long timestampStart = System.currentTimeMillis();
                    measure(values);
                    long timestampEnd = System.currentTimeMillis();
                    sendMeasure(timestampStart, (int) (timestampEnd - timestampStart), values);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                Thread.sleep(measurePollDelay);
            }
        } catch(InterruptedException ex){
            ex.printStackTrace();
        }
    }
}

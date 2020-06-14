package bio.showme;

public interface MeasureListener {

    void measured(long timestamp,int measureDelayMs,int sensorId,int[] measure);

}

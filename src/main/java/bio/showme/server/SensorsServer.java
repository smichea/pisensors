package bio.showme.server;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;

import bio.showme.MeasureListener;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

public class SensorsServer extends WebSocketServer implements MeasureListener {

    public SensorsServer( int port ) {
        super( new InetSocketAddress( port ) );
    }

    @Override
    public void onOpen( WebSocket conn, ClientHandshake handshake ) {
        conn.send("Welcome to the server!"); //This method sends a message to the new client
        broadcast( "new connection: " + handshake.getResourceDescriptor() ); //This method sends a message to all clients connected
        System.out.println( conn.getRemoteSocketAddress().getAddress().getHostAddress() + " entered the place!" );
    }

    @Override
    public void onClose( WebSocket conn, int code, String reason, boolean remote ) {
        broadcast( conn + " has left the place!" );
        System.out.println( conn + " has left the place!" );
    }

    @Override
    public void onMessage( WebSocket conn, String message ) {
        broadcast( message );
        System.out.println( conn + ": " + message );
    }
    @Override
    public void onMessage( WebSocket conn, ByteBuffer message ) {
        broadcast( message.array() );
        System.out.println( conn + ": " + message );
    }

    @Override
    public void onError( WebSocket conn, Exception ex ) {
        ex.printStackTrace();
        if( conn != null ) {
            // some errors like port binding failed may not be assignable to a specific websocket
        }
    }

    @Override
    public void onStart() {
        System.out.println("Server started!");
        setConnectionLostTimeout(0);
        setConnectionLostTimeout(100);
    }

    public void measured(long timestamp,int measureDelayMs,int sensorId,int[] measure){
        String message=timestamp+";"+measureDelayMs+";"+sensorId+";";
        String sep="";
        for(int m:measure){
            message+=sep+m;
            sep=",";
        }
        broadcast(message);
    }

}
package zebrajaeger.de.bedapp;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.UUID;

/**
 * Created by Lars Brandt on 18.06.2017.
 */
public class BT {
    public static final String LOG_TAG = "BT";
    public static final BT I = new BT();
    private static final UUID SerialPortServiceClass_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private final BluetoothAdapter bta;
    private BluetoothSocket socket = null;
    private BluetoothDevice currentDevice = null;
    private Map<String, BluetoothDevice> devices = Collections.EMPTY_MAP;

    private BT() {
        bta =  BluetoothAdapter.getDefaultAdapter();
    }

    private String createUniqueName(BluetoothDevice d) {
        return d.getName() + "(" + d.getAddress() + ")";
    }

    public void refreshDeviceList() {
        devices = new HashMap<>();
        if (bta != null) {
            for (BluetoothDevice d : bta.getBondedDevices()) {
                devices.put(createUniqueName(d), d);
            }
        }
    }

    public ArrayList<String> getSortedDeviceNames() {
        return new ArrayList(new TreeSet(devices.keySet()));
    }

    public String[] getSortedDeviceNamesAsArray() {
        Set<String> names = devices.keySet();
        return new TreeSet<>(names).toArray(new String[names.size()]);
    }

    public List<BluetoothDevice> getDevices() {
        List<BluetoothDevice> result = new ArrayList<>(devices.size());
        for (String name : getSortedDeviceNames()) {
            result.add(devices.get(name));
        }
        return result;
    }

    public boolean setCurrentDevice(String name) {
        BluetoothDevice dev = devices.get(name);
        return setCurrentDevice(dev);
    }

    public boolean setCurrentDevice(BluetoothDevice currentDevice) {
        if (this.socket != null) {
            if (this.socket.isConnected()) {
                try {
                    this.socket.close();
                } catch (IOException e) {
                    Log.e("BT", "Could not close socket: " + e, e);
                }
            }
            this.socket = null;
        }

        this.currentDevice = currentDevice;
        if (this.currentDevice != null) {
            try {
                socket = this.currentDevice.createRfcommSocketToServiceRecord(SerialPortServiceClass_UUID);
                socket.connect();
            } catch (IOException e) {
                Log.e(LOG_TAG, "Could not create and connect socket: " + e, e);
            }
        }

        if (!isConnected()) {
            Log.e(LOG_TAG, "Socket is not connected, clean up");
            socket = null;
            this.currentDevice = null;
        }
        return currentDevice != null;
    }

    public boolean isConnected() {
        return currentDevice != null && socket != null && socket.isConnected();
    }

    public void send(byte value) throws IOException {
        if(isConnected()){
            socket.getOutputStream().write(value);
        }
    }
}

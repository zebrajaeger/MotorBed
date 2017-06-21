package zebrajaeger.de.bedapp;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;

import org.apache.commons.lang3.StringUtils;

import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import zebrajaeger.de.bedapp.storage.AppData;
import zebrajaeger.de.bedapp.storage.Storage;
/**
 * Created by Lars Brandt on 18.06.2017.
 */
public class MainActivity extends AppCompatActivity {

    public static final int TIMER_PERIOD = 100;
    public static final String LOG_TAG = "BT-Main";
    private final AtomicBoolean up = new AtomicBoolean(false);
    private final AtomicBoolean down = new AtomicBoolean(false);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        btAutoconnect();

        Timer timer = new Timer();
        timer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                try {
                    if (up.get() && !down.get()) {
                        BT.I.send((byte) 'u');
                    } else if (!up.get() && down.get()) {
                        BT.I.send((byte) 'd');
                    } else {
                        BT.I.send((byte) 'w');
                    }
                } catch (IOException e) {
                    Log.e(LOG_TAG, "Could not send message" + e, e);
                }
            }
        }, 0, TIMER_PERIOD);

        findViewById(R.id.button_up).setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent event) {
                if (event.getAction() == MotionEvent.ACTION_DOWN) {
                    up.set(true);
                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    up.set(false);
                }
                return true;
            }
        });

        findViewById(R.id.button_down).setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent event) {
                if (event.getAction() == MotionEvent.ACTION_DOWN) {
                    down.set(true);
                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    down.set(false);
                }
                return true;
            }
        });
    }

    public boolean btAutoconnect() {
        AppData appData = Storage.I.getAppData(getApplicationContext());
        BT.I.refreshDeviceList();
        String btAdapter = appData.getBtAdapter();
        if (BT.I.getSortedDeviceNames().contains(btAdapter)) {
            if (BT.I.setCurrentDevice(btAdapter)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            buttonBtSelectDeviceOnClick();
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    public void buttonBtSelectDeviceOnClick() {
        BT.I.refreshDeviceList();
        final String[] names = BT.I.getSortedDeviceNamesAsArray();

        boolean hasDevices = (names.length > 0);
        final AtomicInteger selected = new AtomicInteger(hasDevices ? 0 : -1);
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle(hasDevices ? "Select Bluetooth Device" : "No Devices Found");
        if (hasDevices) {

            // find index of already selected device if exists
            int checked = 0;
            String btAdapter = Storage.I.getAppData().getBtAdapter();
            if (StringUtils.isNotBlank(btAdapter)) {
                int pos = 0;
                for (String n : names) {
                    if (btAdapter.equals(n)) {
                        checked = pos;
                    }
                    ++pos;
                }
            }

            // title
            builder.setTitle("Select Bluetooth Device");

            // devices
            builder.setSingleChoiceItems(names, checked, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    selected.set(which);
                }
            });

            // OK button
            builder.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    String name = names[selected.get()];
                    Storage.I.getAppData(getApplicationContext()).setBtAdapter(name);
                    Storage.I.save(getApplicationContext());
                    BT.I.setCurrentDevice(name);
                }
            });

            // CANCEL button
            builder.setNegativeButton("Cancel", null);
        } else {

            // no devices found
            builder.setTitle("No Devices Found");
            builder.setPositiveButton("Ok", null);
        }

        AlertDialog alertDialog = builder.create();
        alertDialog.show();
    }
}

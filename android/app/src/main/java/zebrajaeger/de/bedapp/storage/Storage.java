package zebrajaeger.de.bedapp.storage;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.google.gson.Gson;

import java.util.Objects;

/**
 * @author Lars Brandt on 18.06.2017.
 */
public class Storage {
    public static final Storage I = new Storage();
    private AppData appData;


    private SharedPreferences getPreferences(Context con) {
        return PreferenceManager.getDefaultSharedPreferences(con);
    }

    public AppData getAppData() {
        return appData;
    }

    public AppData getAppData(Context con) {
        if (appData == null) {
            appData = load(con, AppData.class);
            if (appData == null) {
                appData = new AppData();
            }
        }
        return appData;
    }

    public void save(Context con) {
        if (appData != null) {
            save(con, appData);
        }
    }

    protected void save(Context con, Object o) {
        SharedPreferences.Editor prefsEditor = getPreferences(con).edit();
        prefsEditor.putString(o.getClass().getName(), new Gson().toJson(o));
        prefsEditor.commit();
    }

    protected <T> T load(Context con, Class<T> clazz) {
        Objects.requireNonNull(con, "Context");
        String json = getPreferences(con).getString(clazz.getName(), null);
        return (json != null) ? new Gson().fromJson(json, clazz) : null;
    }
}

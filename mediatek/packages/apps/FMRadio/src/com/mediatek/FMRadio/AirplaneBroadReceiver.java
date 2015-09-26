package com.mediatek.FMRadio;
import android.content.SharedPreferences;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
import com.mediatek.FMRadio.FMRadioActivity;
import android.app.ActivityManager;
import java.util.List;
import android.content.ComponentName;
import android.widget.Toast;

public class AirplaneBroadReceiver extends BroadcastReceiver
{
	private static final String TAG = "Airplane_receiver";
        private static final String AIRPLANE_NAME = "Airplane";
         private static final String FMRadioService_name="com.mediatek.FMRadio.FMRadioService";
        private static final String FLAG_AIRPLANE_NAME = "FAirplane";
      // protected boolean mAirplaneModeEnabled = false; 
        private FMRadioService mService = null;
	private Context context = null;

        public static boolean isServiceExisted(Context context, String className) {
        ActivityManager activityManager = (ActivityManager) context.getSystemService(Context.ACTIVITY_SERVICE);
        List<ActivityManager.RunningServiceInfo> serviceList = activityManager.getRunningServices(Integer.MAX_VALUE);

        if(!(serviceList.size() > 0)) {
            return false;
        }

        for(int i = 0; i < serviceList.size(); i++) {
            ActivityManager.RunningServiceInfo serviceInfo = serviceList.get(i);
            ComponentName serviceName = serviceInfo.service;

            if(serviceName.getClassName().equals(className)) {
                return true;
            }
        } 
        return false;
        }
      public void onReceive(Context context, Intent intent) {
             String action = intent.getAction();
               // LogUtils.d(TAG, "mAirplaneModeEnabled"+mAirplaneModeEnabled);
             // mService.powerDownAsync();
         
        if (Intent.ACTION_AIRPLANE_MODE_CHANGED.equals(action)) {
                FMRadioActivity.mAirplaneModeEnabled = intent.getBooleanExtra("state", false);
               // powerDownFM();
                 LogUtils.d(TAG, "isServiceExisted"+isServiceExisted(context,FMRadioService_name));
                  if((FMRadioActivity.mAirplaneModeEnabled==true)&&(isServiceExisted(context,FMRadioService_name)==true))
                  {
                   Toast.makeText(context, R.string.Airplan_text, Toast.LENGTH_LONG).show();
                   context.stopService(new Intent(context, FMRadioService.class));
                  }
                 SharedPreferences sharedPreferences =context.getSharedPreferences(AIRPLANE_NAME,0);
                 SharedPreferences.Editor editor = sharedPreferences.edit();
                 editor.putBoolean(FLAG_AIRPLANE_NAME, FMRadioActivity.mAirplaneModeEnabled);
                 editor.commit();
                LogUtils.d(TAG, "mAirplaneModeEnabled"+FMRadioActivity.mAirplaneModeEnabled);
                return;
            }
 
    
     }

}

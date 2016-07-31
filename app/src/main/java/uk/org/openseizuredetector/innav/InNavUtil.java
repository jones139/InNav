package uk.org.openseizuredetector.innav;

import android.content.Context;
import android.os.Handler;
import android.widget.Toast;

/**
 * Created by graham on 31/07/16.
 */
public class InNavUtil {
    Context mContext;
    Handler mHandler;

    InNavUtil(Context context, Handler handler) {
        mContext = context;
        mHandler = handler;
    }

    /**
     * used to make sure timers etc. run on UI thread
     */
    public void runOnUiThread(Runnable runnable) {
        mHandler.post(runnable);
    }


    /**
     * Display a Toast message on screen.
     *
     * @param msg - message to display.
     */
    public void showToast(final String msg) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(mContext, msg,
                        Toast.LENGTH_LONG).show();
            }
        });
    }


}

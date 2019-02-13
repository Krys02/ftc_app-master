package com.opencv.checkmatecv;

import android.app.Activity;
import android.content.Context;
import android.view.View;

public class ActivityViewDisplay implements ViewDisplay {

    //There should only be one instance of this class, so make a static reference to it
    private static ActivityViewDisplay instance;
    private static View main = null;

    private ActivityViewDisplay() {
    }

    public static ActivityViewDisplay getInstance() {
        if (instance == null) instance = new ActivityViewDisplay();
        return instance;
    }

    /**
     * Sets this display to be the current one in use, and starts it on the UI thread (as opposed to the robot controller thread)
     * @param context The context of the OpMode, can be obtained via hardwaremap.appContext;
     * @param view The view upon which this activity is to be displayed
     */
    public void setCurrentView(final Context context, final View view) {
        final Activity activity = (Activity) context;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (main == null)
                    main = activity.getCurrentFocus();
                activity.setContentView(view);
            }
        });
    }

    public void removeCurrentView(final Context context) {
        final Activity activity = (Activity) context;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                activity.setContentView(main.getRootView());
            }
        });
    }
}

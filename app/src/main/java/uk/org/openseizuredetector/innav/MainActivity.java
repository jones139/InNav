package uk.org.openseizuredetector.innav;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private final float N2S=1000000000f;   // factor to convert seconds to nanoseconds (1e9)

    private final int APP_MODE_CAL = 2;
    private final int APP_MODE_NAV = 3;

    private int mAppMode = APP_MODE_NAV;

    SensorManager mSensorManager;
    Sensor mAcc,mMag,mGyro;
    InNavUtil mUtil;
    INS mINS;

    //Initial Pos/Att
    private float[] INIPos={0,0,5};
    private float[] INIVel={0,0,0};
    private float[] INICbn={1,0,0,0,1,0,0,0,1};

    //Most recent sensor data and its timestamp
    private float[] dAcc=new float[3];
    private float[] dGyro=new float[3];
    private float[] dMag=new float[3];
    private long etime=0; //Time for the latest sensor data
    private long ptAcc=0,ptGyro=0,ptMag=0;	//previous Sample Time

    //Sensor Calibration values
    private float[] cBAcc=new float[3];
    private float[] cBGyro=new float[3];

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mUtil = new InNavUtil(this,new Handler());
        mINS=new INS(INIPos, INIVel, INICbn);

        int rate = SensorManager.SENSOR_DELAY_FASTEST;
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMag = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorManager.registerListener(this, mAcc , rate);
        mSensorManager.registerListener(this, mMag , rate);
        if (mGyro!=null) {
            mUtil.showToast("registered gyroscope");
            mSensorManager.registerListener(this, mGyro, rate);
        }
        else
            mUtil.showToast("No Gyroscope Available");
    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        int etype = event.sensor.getType();
        etime = event.timestamp;
        float dt=0;
        //Recod the value and time and save as local class variables dAcc, dGyro and dMag
        switch (etype) {
            case Sensor.TYPE_ACCELEROMETER:
                dAcc[0]=event.values[0]-cBAcc[0];
                dAcc[1]=event.values[1]-cBAcc[1];
                dAcc[2]=event.values[2]-cBAcc[2];
                if (ptAcc!=0) dt=(etime-ptAcc)/N2S;
                ptAcc=etime;
                break;
            case Sensor.TYPE_GYROSCOPE:
                dGyro[0]=event.values[0]-cBGyro[0];
                dGyro[1]=event.values[1]-cBGyro[1];
                dGyro[2]=event.values[2]-cBGyro[2];
                if (ptGyro!=0) dt=(etime-ptGyro)/N2S;
                ptGyro=etime;
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                dMag[0]=event.values[0];
                dMag[1]=event.values[1];
                dMag[2]=event.values[2];
                if (ptMag!=0) dt=(etime-ptMag)/N2S;
                ptMag=etime;
                break;
        }
        if (mAppMode==APP_MODE_CAL) // calibration
            break;
        else if (mAppMode==APP_MODE_NAV) {	//6Dof Calculations
            if (etype==Sensor.TYPE_ACCELEROMETER) { //Update velocity and Pos
                //Update pos and vel
                mINS.update_velI(dAcc, dt);
                mINS.update_posII(dt);

                //Update acc accum
                mINS.accum.addacc(dAcc);
            }

            if (etype==Sensor.TYPE_GYROSCOPE) { //Update velocity and Pos
                //Update pos and vel
                mINS.update_attI(dGyro, dt);
                mINS.update_velII(dGyro, dt);
                mINS.update_posI(dGyro, dt);

                //Update acc accum
                mINS.accum.addgyro(dGyro);
            }

            //Set the camera pos & orientation for cube
            mCube.set_dcm(mINS.get_dcm());
            mCube.set_pos(mINS.get_pos());

            //State Updates and Covariance propagation
            if (etime>tProp || ZFlag || CFlag) {
                //First update (propagate the covariance to the current time)
                dt=(etime-ptProp)/N2S;
                ptProp=etime;

                //Propagate the covariance
                mKalman.Propagate(mINS, dt);

                //Clear sensor data accumulators
                mINS.accum.clear();

                //Next Covaraince update time
                tProp=etime+PERPROP;

                //Debug screen
                etv.setText("Pos :" + mINS.Pos_b.data[0] + "\n" + mINS.Pos_b.data[1] + "\n"+ mINS.Pos_b.data[2] + "\n" +
                        "Vel :" + mINS.Vel_b.data[0] + "\n" + mINS.Vel_b.data[1] + "\n"+ mINS.Vel_b.data[2]);
                //etv.setText("Abias :" + cBAcc[0] + "\n" + cBAcc[1] + "\n"+ cBAcc[2] + "\n" +
                //	    "Gbias :" + cBGyro[0] + "\n" + cBGyro[1] + "\n"+ cBGyro[2]);

                //flow_control(4);

                //Check if there is an update request
                if (ZFlag) { //Process Zupt request
                    mKalman.applyZupt(mINS, cBAcc, cBGyro);
                    ZFlag=false;
                }
                if (CFlag) { //Process Cupt Request
                    mKalman.applyCupt(mINS, cBAcc, cBGyro, INIPos);
                    CFlag=false;
                }

            }


    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }


}

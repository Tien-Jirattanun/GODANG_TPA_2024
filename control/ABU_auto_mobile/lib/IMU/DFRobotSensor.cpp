#include <DFRobotSensor.h>

#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266) || defined(ARDUINO_RASPBERRY_PI_PICO)) // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/17, /*tx =*/16);
#define FPSerial softSerial
#else
#define FPSerial Serial2
#endif

DFRobotSensor::DFRobotSensor()
    : sensor(&FPSerial), previousTime(0), vx(0), vy(0), vz(0),
      px(0), py(0), pz(0), roll(0), pitch(0),
      ax_filtered(0), ay_filtered(0), az_filtered(0),
      gx_filtered(0), gy_filtered(0), gz_filtered(0),
      tx(0.0), ty(0.0), tz(0.0), set_zero(0.0), angle_z(0.0) {}

void DFRobotSensor::begin()
{

#if (defined ESP32)
    FPSerial.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
    FPSerial.begin(9600);
#endif

    // Serial2.begin(9600);

    // Set the sensor output frequency
    sensor.modifyFrequency(FREQUENCY_200HZ);

    // Reset angle Z
    long d = millis();
    while (millis() - d <= 3000)
    {
        if (sensor.available())
        {
            set_zero = sensor.Angle.Z;
        }
    }

    // Initialize previousTime to the current time
    previousTime = millis();
}

void DFRobotSensor::Reset()
{
    set_zero = sensor.Angle.Z;
}

void DFRobotSensor::update()
{
    if (sensor.available())
    {
        // unsigned long currentTime = millis();
        // delta_time = (currentTime - previousTime) / 1000.0; // Convert ms to seconds
        // previousTime = currentTime;

        // Angle Z process
        if (sensor.Angle.Z >= set_zero)
        {
            angle_z = sensor.Angle.Z - set_zero;
        }
        else
        {
            angle_z = 360 - (set_zero - sensor.Angle.Z);
        }

        // Read raw accelerometer and gyroscope values from the sensor
        // float ax_raw = sensor.Acc.X;
        // float ay_raw = sensor.Acc.Y;
        // float az_raw = sensor.Acc.Z;
        // float gx_raw = sensor.Gyro.X;
        // float gy_raw = sensor.Gyro.Y;
        // float gz_raw = sensor.Gyro.Z;

        // Apply low-pass filter to accelerometer data
        // ax_filtered = alpha * ax_raw + (1 - alpha) * ax_filtered;
        // ay_filtered = alpha * ay_raw + (1 - alpha) * ay_filtered;
        // az_filtered = alpha * az_raw + (1 - alpha) * az_filtered;

        // Apply low-pass filter to gyroscope data
        // gx_filtered = alpha * gx_raw + (1 - alpha) * gx_filtered;
        // gy_filtered = alpha * gy_raw + (1 - alpha) * gy_filtered;
        // gz_filtered = alpha * gz_raw + (1 - alpha) * gz_filtered;

        // Apply complementary filter to calculate roll and pitch
        // float accRoll = atan2(ay_filtered, az_filtered) * 180 / PI;
        // float accPitch = atan2(-ax_filtered, sqrt(ay_filtered * ay_filtered + az_filtered * az_filtered)) * 180 / PI;

        // roll = 0.98 * (roll + gx_filtered * delta_time) + 0.02 * accRoll;
        // pitch = 0.98 * (pitch + gy_filtered * delta_time) + 0.02 * accPitch;

        // Subtract gravity from accelerometer data
        // float ax_world = ax_filtered - sin(pitch * PI / 180.0);
        // float ay_world = ay_filtered + sin(roll * PI / 180.0) * cos(pitch * PI / 180.0);
        // float az_world = az_filtered - cos(roll * PI / 180.0) * cos(pitch * PI / 180.0);

        // Integrate acceleration to get velocity
        // vx += ax_raw * delta_time;
        // vy += ay_raw * delta_time;
        // vz += az_raw * delta_time;

        // Correct for any bias in velocity when known to be stationary
        // if (isXStationary(ax_raw))
        // {
        //     vx = 0;
        //     vz = 0;
        // }
        // if (isYStationary(ay_raw))
        // {
        //     vy = 0;
        //     vz = 0;
        // }

        // Integrate velocity to get position
        // px += vx * delta_time;
        // py += vy * delta_time;
        // pz += vz * delta_time;

        // Apply translation
        // px += tx;
        // py += ty;
        // pz += tz;
    }
}

SensorData DFRobotSensor::getSensorData() const
{
    // SensorData data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sensor.Angle.Z};

    SensorData data = {px, py, pz, sensor.Acc.X, sensor.Acc.Y, sensor.Acc.Z, sensor.Gyro.X, sensor.Gyro.Y, sensor.Gyro.Z, sensor.Angle.X, sensor.Angle.Y, angle_z};
    return data;
}

bool DFRobotSensor::isStationary(float ax, float ay, float az)
{
    float threshold = 0.3; // Threshold for determining stationary state (adjust as needed)
    return (abs(ax) < threshold && abs(ay) < threshold);
}

bool DFRobotSensor::isXStationary(float ax)
{
    float threshold = 0.6; // Threshold for determining stationary state (adjust as needed)
    return (abs(ax) < threshold);
}

bool DFRobotSensor::isYStationary(float ay)
{
    float threshold = 0.4; // Threshold for determining stationary state (adjust as needed)
    return (abs(ay) < threshold);
}

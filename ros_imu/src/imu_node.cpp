#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <serial/serial.h>

bool handle_imu_data(const char raw_data, std::vector<short>& buff,
                     sensor_msgs::Imu& imu_data,
                     sensor_msgs::MagneticField& mag_data, int* key) {
  buff[*key] = raw_data;
  *key += 1;

  // check header
  if (buff[0] != 0x55) {
    *key = 0;
    buff.clear();
    return false;
  }

  // check length
  if (*key < 11) {
    return false;
  }

  // check sum
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += buff[i];
  }
  if ((sum & 0xff) != buff[10]) {
    *key = 0;
    ROS_WARN("Check sum error");
    return false;
  }

  // parse data
  std::vector<double> data(4);
  for (int i = 0; i < 3; i++) {
    data[i] = buff[2 * i + 2] + (buff[2 * i + 3] << 8);
  }

  // check type
  switch (buff[1]) {
    case 0x51: {
      imu_data.linear_acceleration.x = data[0] / 32768.0 * 16 * 9.8;
      imu_data.linear_acceleration.y = data[1] / 32768.0 * 16 * 9.8;
      imu_data.linear_acceleration.z = data[2] / 32768.0 * 16 * 9.8;
      *key = 0;
      return false;
      break;
    }
    case 0x52: {
      imu_data.angular_velocity.x = data[0] / 32768.0 * 2000;
      imu_data.angular_velocity.y = data[1] / 32768.0 * 2000;
      imu_data.angular_velocity.z = data[2] / 32768.0 * 2000;
      *key = 0;
      return false;
      break;
    }
    case 0x53: {
      double angle_roll = data[0] / 32768.0 * M_PI;
      double angle_pitch = data[1] / 32768.0 * M_PI;
      double angle_yaw = data[2] / 32768.0 * M_PI;
      double ca = cos(angle_roll / 2);
      double sa = sin(angle_roll / 2);
      double cb = cos(angle_pitch / 2);
      double sb = sin(angle_pitch / 2);
      double cc = cos(angle_yaw / 2);
      double sc = sin(angle_yaw / 2);
      imu_data.orientation.w = cc * cb * ca - sc * sb * sa;
      imu_data.orientation.x = cc * cb * sa + sc * sb * ca;
      imu_data.orientation.y = cc * sb * ca - sc * cb * sa;
      imu_data.orientation.z = sc * cb * ca + cc * sb * sa;
      *key = 0;
      return true;
      break;
    }
    case 0x54: {
      mag_data.magnetic_field.x = data[0];
      mag_data.magnetic_field.y = data[1];
      mag_data.magnetic_field.z = data[2];
      *key = 0;
      return false;
      break;
    }
    default: {
      *key = 0;
      break;
    }
  }
  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_node");
  int baudrate;
  ros::param::get("~baudrate", baudrate);
  std::string port;
  ros::param::get("~port", port);
  serial::Serial ser;

  try {
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException& e) {
    ROS_ERROR("Unable to open port, port: %s, baudrate: %d", port.c_str(),
              baudrate);
    return -1;
  }
  if (ser.isOpen()) {
    ROS_WARN("Serial Port initialized succesfully, port: %s, baudrate: %d",
             port.c_str(), baudrate);
  } else {
    ser.open();
    ROS_WARN("Serial Port initialized successfully, port: %s, baudrate: %d",
             port.c_str(), baudrate);
  }

  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/sensor/imu", 20);
  ros::Publisher mag_pub =
      nh.advertise<sensor_msgs::MagneticField>("/sensor/mag", 20);
  ros::Rate loop_rate(500);

  int key = 0;
  std::vector<short> buff(11);
  sensor_msgs::MagneticField mag_data;
  sensor_msgs::Imu imu_data;
  while (ros::ok()) {
    ros::Time current_time = ros::Time::now();
    std::string raw_data;
    size_t n = ser.available();
    if (n != 0) {
      raw_data = ser.read(n);
      if (raw_data.size() == 0) {
        ROS_WARN("No data received, is the IMU connected?");
        continue;
      }
      for (int i = 0; i < n; ++i) {
        bool flag =
            handle_imu_data(raw_data[i], buff, imu_data, mag_data, &key);
        if (flag) {
          ROS_INFO("IMU data received");
          imu_data.header.stamp = current_time;
          imu_data.header.frame_id = "base_link";

          mag_data.header.stamp = current_time;
          mag_data.header.frame_id = "base_link";

          imu_pub.publish(imu_data);
          mag_pub.publish(mag_data);
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ser.close();
  return 0;
}
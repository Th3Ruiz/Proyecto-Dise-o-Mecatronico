#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SERVO_PIN 13
#define TRIG_PIN 12
#define ECHO_PIN 14
#define ENCODER_PIN 15

#define ENA 32
#define ENB 33
#define IN1 2
#define IN2 4
#define IN3 16
#define IN4 17

const int VELOCIDAD_LENTA = 200;
const int PULSOS_POR_VUELTA = 1346;
const float DISTANCIA_CM_POR_PULSO = 21.99 / PULSOS_POR_VUELTA;

const int PULSOS_DIAGONAL = 400;
const int PULSOS_RECTO = 350;
const int PULSOS_AJUSTE = 400;
const int PULSOS_AVANCE = 50;

volatile long contador_pulsos = 0;
Servo direccion;
Adafruit_MPU6050 mpu;
bool maniobra_realizada = false;

rcl_publisher_t twist_publisher;
rcl_publisher_t distance_publisher;
rcl_publisher_t imu_publisher;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 distance_msg;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void IRAM_ATTR contarPulsos() {
  contador_pulsos++;
}

void setupMicroROS() {
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot", "", &support);
  rclc_publisher_init_default(&twist_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "model/robot/cmd_vel");
  rclc_publisher_init_default(&distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "distancia");
  rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
}

void setupMPU6050() {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void publishIMUData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  int64_t time_now = rmw_uros_epoch_millis();
  imu_msg.header.stamp.sec = time_now / 1000;
  imu_msg.header.stamp.nanosec = (time_now % 1000) * 1000000;
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;
  imu_msg.angular_velocity.x = g.gyro.x * (M_PI / 180.0);
  imu_msg.angular_velocity.y = g.gyro.y * (M_PI / 180.0);
  imu_msg.angular_velocity.z = g.gyro.z * (M_PI / 180.0);
  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void publishDistance(float distancia) {
  distance_msg.data = distancia;
  rcl_publish(&distance_publisher, &distance_msg, NULL);
}

void publicarVelocidades(float lineal, float angular) {
  twist_msg.linear.x = lineal;
  twist_msg.angular.z = angular;
  rcl_publish(&twist_publisher, &twist_msg, NULL);
}

void moverAdelanteIndefinido() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, VELOCIDAD_LENTA);
  analogWrite(ENB, VELOCIDAD_LENTA);
  publicarVelocidades(0.10, 0.0);
}

void moverAdelanteTiempo(int ms) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, VELOCIDAD_LENTA);
  analogWrite(ENB, VELOCIDAD_LENTA);
  publicarVelocidades(0.10, 0.0);
  delay(ms);
  detenerMotores();
}

void moverAtras(long pulsos_objetivo) {
  contador_pulsos = 0;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDAD_LENTA);
  analogWrite(ENB, VELOCIDAD_LENTA);
  publicarVelocidades(-0.10, 0.0);
  while (contador_pulsos < pulsos_objetivo) {
    publishIMUData();
    delay(10);
  }
  detenerMotores();
}

void detenerMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  publicarVelocidades(0.0, 0.0);
}

float leerDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duracion = pulseIn(ECHO_PIN, HIGH, 20000);
  float distancia = duracion * 0.034 / 2.0;
  publishDistance(distancia);
  return distancia;
}

void fase1RetrocederDiagonal() {
  direccion.write(140);
  publicarVelocidades(-0.10, 0.5);
  moverAtras(PULSOS_DIAGONAL);
}

void fase2RetrocederRecto() {
  direccion.write(90);
  publicarVelocidades(-0.10, 0.0);
  moverAtras(PULSOS_RECTO);
}

void fase3RetrocederAjustando() {
  direccion.write(40);
  publicarVelocidades(-0.10, -0.5);
  moverAtras(PULSOS_AJUSTE);
}

void fase4AvanzarCentrar() {
  direccion.write(90);
  publicarVelocidades(0.10, 0.0);
  moverAdelanteTiempo(1000);
}

void ejecutarSecuenciaEstacionamiento() {
  fase1RetrocederDiagonal();
  fase2RetrocederRecto();
  fase3RetrocederAjustando();
  fase4AvanzarCentrar();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  direccion.attach(SERVO_PIN);
  direccion.write(90);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), contarPulsos, FALLING);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Wire.begin();
  setupMPU6050();
  setupMicroROS();
  detenerMotores();
}

void loop() {
  static unsigned long last_imu_publish = 0;
  if (millis() - last_imu_publish > 100) {
    publishIMUData();
    last_imu_publish = millis();
  }
  if (!maniobra_realizada) {
    float distancia = leerDistancia();
    if (distancia > 0 && distancia < 25) {
      moverAdelanteTiempo(1700);
      detenerMotores();
      ejecutarSecuenciaEstacionamiento();
      maniobra_realizada = true;
    } else {
      moverAdelanteIndefinido();
    }
  } else {
    detenerMotores();
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
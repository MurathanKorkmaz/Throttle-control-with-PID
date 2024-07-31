
# Elektronik Gaz Pedalı ve Elektronik Gaz Kelebeği Kontrol Projesi

Bu proje, Atmega2560 mikrodenetleyici tabanlı elektronik gaz pedalı ve elektronik gaz kelebeği kontrolü üzerine odaklanmaktadır. 
Proje, modern araçlarda performans ve yakıt verimliliğini artırmak için elektronik gaz kelebeği kontrolünü geliştirmeyi amaçlamaktadır.

## Proje Amacı
Elektronik gaz pedalı ve gaz kelebeği sistemleri, sürüş deneyimini optimize etmek ve araç güvenliğini artırmak için kritik öneme sahiptir. 
Bu projede, bu iki bileşen arasındaki etkileşimi hassas bir şekilde kontrol etmek için PID (Oransal, İntegral, Türev) algoritması kullanılmıştır.
Böylece, araç motorunun performansı ve yanıt verme hızı artırılmış, aynı zamanda yakıt verimliliği ve emisyonlar optimize edilmiştir.

## Kullanılan Bileşenler ve Teknolojiler

### Donanım
- Atmega2560 Mikrodenetleyici:  ** Projenin ana kontrol birimi.
- Elektronik Gaz Pedalı (6 Pin): VCC, GND, APPS1, APPS2, Sensor Ground, Reference Voltage ** Sürücünün gaz pedalına uyguladığı basıncı ölçer.
- Elektronik Gaz Kelebeği (6 Pin): VCC, GND, TPS1, TPS2, DC Motor (+), DC Motor (-) ** Motorun hava girişini kontrol eder.

### Yazılım
- Yazılım Dili:** Objective C
- Geliştirme Ortamı:** Microchip Studio
- Kontrol Algoritması:** PID (Oransal, İntegral, Türev) Algoritması
- Diğer Teknolojiler:** ADC (Analog-Dijital Dönüştürücü) ve PWM (Darbe Genişlik Modülasyonu) Sinyalleri

## Proje Detayları
Bu projede, elektronik gaz pedalı tarafından algılanan sürücü girişi, Atmega2560 mikrodenetleyici tarafından işlenir ve uygun PWM sinyalleri üretilir. PID kontrol algoritması, 
gaz kelebeğinin pozisyonunu hassas bir şekilde ayarlayarak motor performansını ve yanıt verme hızını optimize eder. ADC, pedalın konumunu dijital sinyallere dönüştürerek mikrodenetleyiciye iletir.

## Kurulum ve Kullanım
1. Donanım Kurulumu:** Atmega2560 mikrodenetleyiciyi, elektronik gaz pedalı ve gaz kelebeği ile bağlantılandırın.
2. Yazılım Yükleme:** Objective C dilinde yazılmış yazılımı Microchip Studio kullanarak mikrodenetleyiciye yükleyin.
3. Çalıştırma:** Sistemi çalıştırarak gaz pedalı ve gaz kelebeği arasındaki etkileşimi test edin.

## Lisans
Bu proje GNU Genel Kamu Lisansı v2 altında lisanslanmıştır. Daha fazla bilgi için aşağıdaki lisans metnine göz atabilirsiniz.


----------------------------------------------------------------------------------------------------------------------------------


# Electronic Throttle Pedal and Electronic Throttle Control Project

This project focuses on the control of an electronic throttle pedal and electronic throttle body using an Atmega2560 microcontroller. The project aims to enhance electronic throttle control in modern vehicles to improve performance and fuel efficiency.

## Project Purpose
Electronic throttle pedal and throttle body systems are critical for optimizing the driving experience and increasing vehicle safety. In this project, a PID (Proportional, Integral, Derivative) algorithm is used to precisely control the interaction between these two components. Thus, the engine's performance and response time are improved, and fuel efficiency and emissions are optimized.

## Components and Technologies Used

### Hardware
- Atmega2560 Microcontroller:** The main control unit of the project.
- Electronic Throttle Pedal (6 Pins):** VCC, GND, APPS1, APPS2, Sensor Ground, Reference Voltage ** Measures the pressure applied to the gas pedal by the driver.
- Electronic Throttle Body (6 Pins):** VCC, GND, TPS1, TPS2, DC Motor (+), DC Motor (-) ** Controls the air intake of the engine.

### Software
- Programming Language:** Objective C
- Development Environment:** Microchip Studio
- Control Algorithm:** PID (Proportional, Integral, Derivative) Algorithm
- Other Technologies:** ADC (Analog-Digital Converter) and PWM (Pulse Width Modulation) Signals

## Project Details
In this project, the driver input detected by the electronic throttle pedal is processed by the Atmega2560 microcontroller, which generates appropriate PWM signals. The PID control algorithm precisely adjusts the position of the throttle body, optimizing engine performance and response time. The ADC converts the pedal position into digital signals and transmits it to the microcontroller.

## Setup and Usage
1. Hardware Setup:** Connect the Atmega2560 microcontroller with the electronic throttle pedal and throttle body.
2. Software Installation:** Load the software written in Objective C onto the microcontroller using Microchip Studio.
3. Operation:** Test the interaction between the throttle pedal and the throttle body by running the system.

## License
This project is licensed under the GNU General Public License v2. For more information, please refer to the license text below.




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


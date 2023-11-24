# LiDAR
Program do odliczania odległości za pomocą dalkomierza LIDAR, sprawdzania temperatury oraz wyznaczania kątu zasięgu czyli kąt pod jakim pocisk musi zostać wystrzelony aby po przebyciu drogi trafił w punkt.

(Pomijając opór powietrza, krzywiznę Ziemi i inne siły poza siłą grawitacji ciało porusza się po paraboli)

![image](https://github.com/SXC-150/LiDAR/assets/98900646/a93314aa-5e94-453a-9aa8-682cbb05d40e)
![image](https://github.com/SXC-150/LiDAR/assets/98900646/2a4d5cea-8b11-41e3-bc72-b40110473f2f)
![image](https://github.com/SXC-150/LiDAR/assets/98900646/963e4d0a-7da0-4bad-b145-6e31a6739981)![image](https://github.com/SXC-150/LiDAR/assets/98900646/d78f5b2a-47c0-468c-9510-5a4fc9ab5460)
![image](https://github.com/SXC-150/LiDAR/assets/98900646/30ea9236-7bb1-4bf3-aa1a-4724b50b6965)![image](https://github.com/SXC-150/LiDAR/assets/98900646/4af14aec-3045-4158-820b-d616127e1de8)
![image](https://github.com/SXC-150/LiDAR/assets/98900646/f90bc3a8-fadf-474b-a646-982651a46962)

Użyte części w projekcie:

Dalkomierz: Laserowy czujnik odległości Lidar TFmini-S UART/I2C 12m

Wyświetlacz: Wyświetlacz OLED graficzny 0,96'' 128x64px I2C - niebieski

Termometr: Czujnik temperatury i wilgotności DHT11 +50C

Komputer: Arduino Uno Rev3 - A000066 z mikrokontrolerem AVR ATmega328

32 kB pamięci Flash
2 kB pamięci RAM
1 kB pamięci EEPROM
 
Program został napisany w C++ i został wykonany w całości samodzielnie z wyjątkiem części odpowiadającej od wyznaczania odległości i sprawdzania wiarygodności dalkomierza.

Owa część została napisana z pomocą artykułu producenta dalkomierza który można znaleść tutaj: 

https://botland.com.pl/index.php?controller=attachment&id_attachment=2101
 
Autor: Maciej Świerczyński

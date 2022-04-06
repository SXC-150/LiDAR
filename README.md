# LiDAR
Program do odliczania odległości za pomocą dalkomierza LIDAR, sprawdzania temperatury oraz wyznaczania kątu zasięgu czyli kąt pod jakim pocisk musi zostać wystrzelony aby po przebyciu drogi trafił w punkt.

(Pomijając opór powietrza, krzywiznę Ziemi i inne siły poza siłą grawitacji ciało porusza się po paraboli)

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

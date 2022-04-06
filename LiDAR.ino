/*  Program do odliczania odległości za pomocą dalkomierza LIDAR, sprawdzania temperatury oraz wyznaczania kątu zasięgu czyli kąt pod jakim pocisk musi zostać wystrzelony aby po przebyciu drogi trafił w punkt.
 *  (Pomijając opór powietrza, krzywiznę Ziemi i inne siły poza siłą grawitacji ciało porusza się po paraboli)
 *  
 *  Użyte części w projekcie:
 *  Dalkomierz: Laserowy czujnik odległości Lidar TFmini-S UART/I2C 12m
 *  Wyświetlacz: Wyświetlacz OLED graficzny 0,96'' 128x64px I2C - niebieski
 *  Termometr: Czujnik temperatury i wilgotności DHT11 +50C
 *  Komputer: Arduino Uno Rev3 - A000066 z mikrokontrolerem AVR ATmega328
 *  32 kB pamięci Flash
 *  2 kB pamięci RAM
 *  1 kB pamięci EEPROM
 *  
 *  Program został napisany w C++ i został wykonany w całości samodzielnie z wyjątkiem części odpowiadającej od wyznaczania odległości i sprawdzania wiarygodności dalkomierza.
 *  Owa część została napisana z pomocą artykułu producenta dalkomierza który można znaleść tutaj: https://botland.com.pl/index.php?controller=attachment&id_attachment=2101
 *  
 *  Autor: Maciej Świerczyński
 */ 
#include <Arduino.h> // Biblioteka Arduino
// Część graficzna
#include <Wire.h> // Biblioteka interfejsu I2C
#include <U8g2lib.h> // Biblioteka graficzna
#include <LiquidCrystal.h> // Podstawowa biblioteka ekranów LCD (jak na przykład komenda "print" czyli odpowiednik "echo" ale do wyświetlania tekstu na ekranie) 
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_MIRROR); // Ustawianie rodzaju ekranu oraz sposobu wyświetlenia zawartości (Ponieważ pół lustro odbija ekran zniekrztałcony wymagane jest jego wcześniejsze odwrócenie aby pojawiał się prawidłowo)
// Część od wyznaczania odległości i kątu zasięgu
#include <SoftwareSerial.h> // Biblioteka do tworzenia wirtualnych portów seryjnych
#include <Math.h> // Podstawowa biblioteka z funkcjami matematycznymi
#define UARTBUTPIN 4 // PIN Przycisku do włączania pomiaru dalkomierza
SoftwareSerial Serial1(2,3); // ustawnienie wirtualnego portu seryjnego (PIN2 jako RX, PIN3 jako TX)
float GRAW = 9.81; // Przyśpieszenie grawitacyjne
float AMASS = 0.2; // Masa strzały (0.2 kg = 200 gramów)
float FORCE = 115.65; // Siła łuku wyrażana w newtonach
float DRAWL = 0.61; // Długość naciągu łuku w metrach
float DIST5M = 1.99; // Dystans gdzie "kąt zasięgu" jest równy 5 metrom. Wyjaśnienie poniżej
float CALIBRATIONPERCENT = 5.3; // Procent kalibracyjny (530%). Wyjaśnienie poniżej
float AngleOfReachNoSin = 0; // Kąt zasięgu bez asin. Wyjaśnienie poniżej
float AngleOfReachRad = 0; // Kąt zasięgu w radianach. Wyjaśnienie poniżej
float AngleOfReach = 0; // Kąt zasięgu w stopniach. Wyjaśnienie poniżej
float AngleOfReach5m = 0; // Kąt zasięgu w stopniach po odjęciu 5 metrów od celu. Wyjaśnienie poniżej
float AngleOfReach5mDeg = 0; // Kąt zasięgu w stopniach (bez Procentu kalibracyjnego) po odjęciu 5 metrów od celu. Wyjaśnienie poniżej
int check; // Zmienna gdzie będą sprawdzanie "czyste" zawartości dalkomierza
int iurat; // licznik w jakim miejscu są wprowadzanie "czyste" wartości dalkomierza
int uart[9]; // Tabelka gdzie będą przechowywanie "czyste" wartości dalkomierza
const int HEADER = 0x59; // Nagłówek którym posługuje się dalkomierz
float dist; // Dystans zmierzony przez dalkomierz (w centymetrach)
// Część od wyznaczania temperatury
#include <DHT.h> // Biblioteka termometra
#define DHTPIN 5 // PIN na którym będą wysyłane dane z termometru
#define ONOFFDHTPIN 6 // Włącznik/wyłącznik termometru
#define DHTTYPE DHT11 // Ustawianie modelu termometra
DHT dht(DHTPIN, DHTTYPE); // Tworzenie klasy dla termometra
int temp; // zmienna gdzie będą przechowywane wyniki z termometra (w Stopniach Celsjusza)
bool StatDht = false; // Wyświetlanie temperatury domyślnie jest wyłączone
bool DhtDot = true; // Kropka sygnalizująca działanie
// Część od przypisania klawiszy z Numpada (inaczej klawiatura 4x3)
#include <Keypad.h> // Biblioteka od numpada
const byte ROWS = 4; // Cztery szeregi przycisków
const byte COLS = 3; // Trzy kolumny przycisków
char KEYS[ROWS][COLS] = { // Przypisanie wartości poszczególnym przyciskom
  {'#','0','*'},
  {'9','8','7'},
  {'6','5','4'},
  {'3','2','1'}
};
byte ROWPINS[ROWS] = {10, 9, 8, 7}; // Połączenia pinów dla szeregów numpada
byte COLPINS[COLS] = {13, 12, 11}; // Połączenia pinów dla kolumn numpada
Keypad keypad = Keypad( makeKeymap(KEYS), ROWPINS, COLPINS, ROWS, COLS ); // tworzenie klasy dla numpada
// Część od tworzenia obrazu celownika w postaci mapy XBM, kalibracji i zapisywania
#define CROSSHAIR_WIDTH  21 // Szerokość mapy XBM
#define CROSSHAIR_HEIGHT 52 // Wysokość mapy XBM
// Poniewasz ekran jest czytany poziomo mapa XBM została przesunięta o 90 stopni w prawo
static unsigned char CROSSHAIR_BITS[] = { // Obraz celownika w postaci mapy XBM
 0x00,0x0c,0xe0,0x00,0x1e,0xe0,0x00,0x3f,0xe0,0x80,0x73,0xe0,
 0xc0,0xe1,0xe0,0xe0,0xc0,0xe1,0x70,0x80,0xe3,0x38,0x00,0xe7,
 0x1c,0x00,0xee,0x0e,0x00,0xfc,0x07,0x00,0xf8,0x04,0x00,0xf0,
 0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,
 0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,
 0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,
 0x04,0x00,0xe0,0x04,0x0c,0xe0,0x04,0x0c,0xe0,0x04,0x00,0xe0,
 0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,
 0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,
 0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,0x04,0x00,0xe0,
 0x04,0x00,0xf0,0x07,0x00,0xf8,0x0e,0x00,0xfc,0x1c,0x00,0xee,
 0x38,0x00,0xe7,0x70,0x80,0xe3,0xe0,0xc0,0xe1,0xc0,0xe1,0xe0,
 0x80,0x73,0xe0,0x00,0x3f,0xe0,0x00,0x1e,0xe0,0x00,0x0c,0xe0};
int calibrationRight = 0; // Kalibracja celownika w prawo (minusowe wartości oznaczają kalibacje w lewo)
int calibrationUp = 0; // Kalibracja celownika w górę (minusowe wartości oznaczają kalibacje w dół)
bool AdvMode = true; // Domyślnie celownik jest ustawiony w trybie zaawansowanym. Wyjaśnienie poniżej
#include <EEPROM.h> // Biblioteka pamięci EEPROM (Kalibracja zapisana w pamięci EEPROM jest trwała i kalibracja zostanie przywrócona po ponownym podłączeniu do zasilania)
int EEPROMright; // Kalibracja celownika w prawo (minusowe wartości oznaczają kalibeacje w lewo) która została zapisana i zostanie przywócona po ponownym podpięciu do zasilania
int EEPROMup; // Kalibracja celownika w górę (minusowe wartości oznaczają kalibeacje w dół) która została zapisana i zostanie przywócona po ponownym podpięciu do zasilania


void setup(){ // Tutaj komendy zostaną uruchomione tylko raz
  Serial.begin(9600); // Ustaw szybkość transmisji dla Arduino
  Serial1.begin(115200); // Ustaw szybkość transmisji dla wirtualnego portu szeregowego (oraz tym samym dalkomierza podłączonego do tego portu)
  pinMode(UARTBUTPIN, INPUT); // Ustawianie aby ten PIN czytał wartości (inaczej czy prąd jest wysoki czy niski)
  pinMode(ONOFFDHTPIN, INPUT); // Ustawianie aby ten PIN czytał wartości (inaczej czy prąd jest wysoki czy niski)
  u8g2.begin(); // Uruchamianie biblioteki graficznej
  dht.begin(); // Uruchamianie biblioteki termometra
  EEPROM.get(0, EEPROMright); // Czytanie i ustawianie wartości z pamięci EEPROM dla zmiennej "EEPROMright"
  EEPROM.get(1, EEPROMup); // Czytanie i ustawianie wartości z pamięci EEPROM dla zmiennej "EEPROMup"
}

void KEYPADoptionsANDdht(){ // Funkcja z opcjami dostępnymi dla numpada oraz sprawdzanie temperatury
  char key = keypad.getKey(); // Czytanie wciśniętego klawisza
  if (digitalRead(ONOFFDHTPIN) == LOW || key == '9'){ // Jeżeli przełącznik ONOFFDHTPIN zostanie przesunięty na pozycje "wyłącz" LUB zostanie wciśnięty klawisz "9"...
    if ((millis()/1000) % 2 == 0 || key == '9'){ // Jeżeli liczba sekund od czasu włączenia urządzenia jest parzysta LUB zostanie wciśnięty klawisz "9" (nie ma potrzeby wciskania klawiszu "9" dwa razy)...
      /*  Termometr potrzebuje około jednej sekundy pauzy aby móc wykonać dokładny pomiar
       *  Dlatego termometr aktywowany jest wtedy kiedy liczba sekund (od uruchomienia urządzenia) jest parzysta LUB tylko raz kiedy wciśnięty zostanie klawisz "9"
       *  Tym samym sygnalizuje czy temperatura jest aktualna
       */
      temp = dht.readTemperature(); // Czytanie temperatury z termometru
      StatDht = true; // Włączanie wyświetlania temperatury
      DhtDot = true; // Sygnalizowanie działania termometru
    }
    else{ // Jeżeli liczba sekund od czasu włączenia urządzenia NIE jest parzysta...
      DhtDot = false; // Sygnalizowanie działania termometru
    }
  } else if (key == '7'){ // Jeżeli klawisz "7" zostanie wciśnięty...
    StatDht = false; // Wyłączanie wyświetlania temperatury
  }
  switch(key){ // Reszta przycisków
    case '1':{
      AdvMode = false; // Jeżeli klawisz "1" zostanie wciśnięty wyłącz tryb zaawansowany. Wyjaśnienie poniżej
    } break;
    case '2':{
      calibrationUp++; // Jeżeli klawisz "2" zostanie wciśnięty karibruj w górę
    } break;
    case '3':{
      AdvMode = true; // Jeżeli klawisz "3" zostanie wciśnięty włącz tryb zaawansowany. Wyjaśnienie poniżej
    } break;
    case '4':{
      calibrationRight--; // Jeżeli klawisz "4" zostanie wciśnięty karibruj w lewo
    } break;
    case '5':{
      calibrationRight=0;  // Jeżeli klawisz "5" zostanie wciśnięty resetuj początkową kalibracje
      calibrationUp=0;
    } break;
    case '6':{
      calibrationRight++; // Jeżeli klawisz "6" zostanie wciśnięty karibruj w prawo
    } break;
    case '8':{
      calibrationUp--; // Jeżeli klawisz "8" zostanie wciśnięty karibruj w dół
    } break;
    case '0':{
      dist=0; // Jeżeli klawisz "0" zostanie wciśnięty resetuj dystans dalkomierza
      AngleOfReach5m=0; // Wyjaśnienie poniżej
    } break;
    case '*':{
      EEPROMright = EEPROMright + calibrationRight; // Jeżeli klawisz "*" zostanie wciśnięty zapisz kalibracje do pamięci EEPROM
      EEPROMup = EEPROMup + calibrationUp;
      EEPROM.put(0, EEPROMright);
      EEPROM.put(1, EEPROMup);
      calibrationRight=0;
      calibrationUp=0;
    } break;
    case '#':{
      for (int i = 0 ; i < EEPROM.length() ; i++) { // Jeżeli klawisz "#" zostanie wciśnięty wykonaj totalny reset pamięci EEPROM i resetuj kalibracje początkową (jeżeli istnieje)
        EEPROM.write(i, 0);
      }
      EEPROMright=0;
      EEPROMup=0;
      calibrationRight=0;
      calibrationUp=0;
    } break;
  }
}

void LIDARdistanceCalculation(){ // Funkcja kalkulowania dystansu i wyznaczania kątu zasięgu
// Ta część została napisana z pomocą artykułu producenta dalkomierza który można znaleść tutaj: https://botland.com.pl/index.php?controller=attachment&id_attachment=2101
  if(Serial1.available()){ // Sprawdź czy stworzony port seryjny wysyła jakieś dane
    if(Serial1.read() == HEADER){ // Jeżeli odczytany sygnał równa się nagłówki...
      uart[0] = HEADER; // Ustaw pierwsze miejsce w tabeli jako nagłówek
      if(Serial1.read() == HEADER){ // Jeżeli odczytany sygnał równa się nagłówki...
        uart[1] = HEADER; // Ustaw drugie miejsce w tabeli jako nagłówek
        for(iurat=2;iurat<9;iurat++){ // Pętla która wprowadza dane z dalkomierza do tabelki
           uart[iurat] = Serial1.read(); // Czytanie z portu seryjnego i wprowadzanie odczytanej wartości do komórki w tabelce. Miejsce w tabeli dyktuje zmienna "iuart"
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7]; // Wprowadzanie i dodawanie wszystkich z wyjątkiem jednego danych do zmiennej "check" w celu sprawdzenia ich dokładności i wiarygodności
        if(uart[8] == (check&0xff)){ // Sprawdzanie otrzymanych danych
           dist = uart[2] + uart[3] * 256; // Kalkulowanie dystansu
        }
      }
    }
  }
// Koniec części
  AngleOfReachNoSin = ((AMASS * GRAW * (dist/100)) / (2 * (FORCE * DRAWL))); // (dist/100) Czyli konwersja cm na m    //              Wzór na kąt zasięgu:
  AngleOfReachRad = (asin(AngleOfReachNoSin) / 2);                                                                    //                     1      AMASS * GRAW * dist    AMASS = Masa strzały w kg     GRAW  = Przyśpieszenie grawitacyjne (9.81m/s)
  AngleOfReach = (AngleOfReachRad * 180 / M_PI); // Konwersja radianów na stopnie                                     //   AngleOfReachRad = - asin -------------------    dist  = Dystans w metrach     FORCE = Siła łuku wyrażana w newtonach
  if (AngleOfReach <= DIST5M){ // Jeżeli dystans zmierzony za pomocą dalkomierza NIE przekracza 5 metrów...           //                     2       2 (FORCE * DRAWL)     DRAWL = Długość naciągu łuku w metrach
    AngleOfReach5m = 0; // Wyzeruj kąt zasięgu                                                                        // Łuk został skalibrowany na odległość 5 metrów (tym samym kropka na celowniku jest punktem wyjściowym)
    AngleOfReach5mDeg = 0;                                                                                            // Skoro wiemy gdzie poleci strzała 5m od celu (oraz niema sensu obliczania kątu dla odległości mniejszej od 5m)...
  } else if (AngleOfReach >= DIST5M){ // Jeżeli dystans zmierzony za pomocą dalkomierza jest WIĘKSZY od 5 metrów...   // Program będzie liczyć kąt zasięgu tylko jeżeli odległość jest większa od 5m oraz odejmujemy kąt zasięgu o wartości 5m (czyli od punktu wyjściowego)
    AngleOfReach5m = (AngleOfReach - DIST5M) * CALIBRATIONPERCENT; // Tutaj jeden stopnień ma wartość 5.3 pikselów na ekranie dlatego każdy stopień został pomnożony o procent kalibracyjny czyli 530%
    AngleOfReach5mDeg = (AngleOfReach - DIST5M); // "AngleOfReach5mDeg" jest tym samym co "AngleOfReach5m" ale bez procentu kalibracyjnego
  }
}

void loop(){ // Tutaj komendy zostaną uruchomione w kółko aż do odłączenia urządzenia od zasilania
  u8g2.firstPage(); // Rozpoczęcie tworzenia "strony" dla panelu LCD     funkcja wewnatrz u8g2 inaczej lagi
  do {
  /*  Pamięć RAM wykorzystywanego urządzenia to jedynie 2048 kb a to zdecydowanie za mało aby odświeżać cały ekran za każdym razem...
   *  Dlatego biblioteka graficzna umożliwia na podzielenie ekranu na 4 części które wczytuje po jedna za drugą w pętli "u8g2.firstPage();" tym samym znacząco redukując wykorzystanie pamięci RAM.
   *  W zamian za to bardziej obciążony jest procesor, rzadko występują szarpania ekranu...
   *  (na szczęście tylko wtedy kiedy wciśnięty będzie czerwony przycisk aby wyznaczyć odległość i kąt zasięgu a nie podczas strzelania z łuku).
   *  Oraz każda instrukcja zmieniająca zmienne które następnie zostaną wyświetlone na ekranie MUŚI znajdować się w pętli "u8g2.firstPage();".
   *  Inaczej komputer pracuje bardzo powolnie i liczby nachodzą na siebie (na przykład 1 jest wewnątrz 0 ponieważ przed chwilą zmierzono odległość zmieniając zmienną "dist")
   *  Ponieważ prawie wszystkie zmienne (wliczając zmienne które zmieniają inne zmienne wyświetlane na ekranie jak np. "dist = uart[2] + uart[3] * 256;") zostaną wyświetlone na ekranie wszystkie komendy w pętli "void loop(){" są równiesz w pętli "u8g2.firstPage();".
   *  Nie zauważyłem aby to rozwiązanie miało negatywny wpływ na wydajność a wrecz przeciwnie!
   */ 
    KEYPADoptionsANDdht(); // Wywołanie funkcji z opcjami dla numpada i sprawdzaniem temperatury
    if (digitalRead(UARTBUTPIN) == HIGH){ // Jeżeli czerwony przycisk zostanie wciśnięty...
      LIDARdistanceCalculation(); // Wywołanie funkcji do mierzenia dystansu oraz obliczania kątu wystrzału
    }
    u8g2.setFont(u8g2_font_5x7_tr); // Ustaw czcionke (szerokość 5 pikseli, wysokość 7 pikseli)
    u8g2.setFontDirection(1); // Odwrót czionki o 90 stopni (aby ekran pojawiał się poziomo nie pionowo)
    if (StatDht == true && temp != 0){ // Jeżeli przełącznik ONOFFDHTPIN zostanie przesunięty na pozycje "wyłącz" LUB klawisz "9" zostanie wciśnięty ORAZ temperatura NIE przekracza najniższej wartości (-1'C) LUB jeżeli miernik NIE napotkał błędu...
      u8g2.drawStr(72+calibrationUp-AngleOfReach5m+EEPROMup, 36+calibrationRight+EEPROMright, "C"); // Wypisz literę "C"
      if (DhtDot == true){ // Jeżeli termometr pobiera tęperaturę...
        u8g2.drawPixel(78+calibrationUp-AngleOfReach5m+EEPROMup, 34+calibrationRight+EEPROMright); 
        /* Zrób kropkę obok litery "C" tak aby wyglądała jak znak Celsjusza
         * Znalazłem te rozwiązanie na bardziej wydajne niż użycie Unicodu zwłaszcza że program potrzebuje jedynie wypisać jedną literę z tego zestawu znaków
         * Oraz ten sposób pozwala na sprawdzanie stanu termometra (czy pobiera tęperaturę czy nie) za pomocą migającej kropki
         */
      }
      u8g2.setCursor(72+calibrationUp-AngleOfReach5m+EEPROMup, 23+calibrationRight+EEPROMright); // Ustaw "kursor" wyprowadzania znaków
      u8g2.print(temp); // Wypisz temperaturę
    }
    if (calibrationRight != 0 || calibrationUp != 0){ // Jeżeli występuje jakakolwiek wartość kalibracyjna...
      u8g2.setCursor(92+calibrationUp-AngleOfReach5m+EEPROMup, 34+calibrationRight+EEPROMright); // Ustaw "kursor" wyprowadzania znaków
      u8g2.print(calibrationUp); // Wypisz wartość kalibracyjną do góry (albo minusową do dołu)
      u8g2.setCursor(82+calibrationUp-AngleOfReach5m+EEPROMup, 34+calibrationRight+EEPROMright); // Ustaw "kursor" wyprowadzania znaków
      u8g2.print(calibrationRight); // Wypisz wartość kalibracyjną w prawo (albo minusową w lewo)
      u8g2.setFont(u8g2_font_unifont_t_75);
      /* Ustaw czcionke Unicode (a bardziej jej małą część ze znakami)
       * W przeciwieństwie do powyszego przykładu z Celsjuszem użycie tutaj funkcji do rysowania trójkątów lub stworzenie grafik XBM dla strzałek jest bardziej wymagające dla komputera
       * Niż wypisanie ich z Unikodu zwłaszcza że owe strzałki pokazują się tylko podczas kalibracji celownika
       */
      u8g2.drawGlyph(89+calibrationUp-AngleOfReach5m+EEPROMup, 24+calibrationRight+EEPROMright, 0x25b3); // Wypisz strałkę do góry
      u8g2.drawGlyph(79+calibrationUp-AngleOfReach5m+EEPROMup, 24+calibrationRight+EEPROMright, 0x25b7); // Wypisz strałkę w prawo
    }
    if (AdvMode == true){ 
      /* Jeżeli tryb zaawansowany jest włączony...
       *  "Tryb Zaawansowany" to inaczej tryb gdzie jest narysowany celownik na mapie XBM, odległość zmierzona za pomocą dalkomierza oraz ile stopni celownik został odsuniędy do dołu
       */
      u8g2.drawXBM(52+calibrationUp-AngleOfReach5m+EEPROMup, 6+calibrationRight+EEPROMright, CROSSHAIR_WIDTH, CROSSHAIR_HEIGHT, CROSSHAIR_BITS);
      u8g2.setFont(u8g2_font_luRS08_tr); // Ustaw czcionke podobną do wcześniejszej ale z lepszym formatowaniem oraz z "trochę ładniejszymi liczbami"
      u8g2.setCursor(44+calibrationUp-AngleOfReach5m+EEPROMup, 23+calibrationRight+EEPROMright); // Ustaw "kursor" wyprowadzania znaków
      if (dist >= 0 && dist < 1500){ // Jeżeli odmierzony dystans mieści się w granicach od 0m do 14.99m...
        u8g2.print(dist/100); // Wypisz odległość (oraz zamień ją z centymetrów na metry)
      } else { // Jeżeli nie albo jeżeli dalkomierz napotkał problem z obliczaniem dystansu lub zmierzony dystans jest nie wiarygodny (dalkomierz podaje błąd wiarygodności dystansu za pomocą wartości ujemnych)...
        u8g2.print("?.???"); // Wypisz znaki zapytania symbolizujące błąd odczytu odległości
      }
      if (AngleOfReach5m != 0){ // Jeżeli odmierzony dystans jest większy od 5 metrów...
        u8g2.setFont(u8g2_font_5x7_tr); // Ustaw czcionke (szerokość 5 pikseli, wysokość 7 pikseli)
        u8g2.setCursor(54+calibrationUp-AngleOfReach5m+EEPROMup, 25+calibrationRight+EEPROMright); // Ustaw "kursor" wyprowadzania znaków
        u8g2.print(AngleOfReach5mDeg); // Wypisz "kąt zasięgu" a bardziej o ile stopni celownik został przesunięty w dół
      }
    } else if (AdvMode == false){ // Inanczej jeżeli tryb zaawansowany jest wyłączony...
      u8g2.drawBox(62+calibrationUp-AngleOfReach5m+EEPROMup, 31+calibrationRight+EEPROMright, 2, 2);
      /* Narysuj jedynie "kropkę" 2 na 2 piksele
       * Jest to bardziej wydajne rozwiązanie oraz pomimo nie wyświetlania odległości w tym trybie nadal jest ona mierzona a "kąt zasięgu" bedzie nadal miał wpływ na przesunięcie kropki w dół
       */
      }
  } while ( u8g2.nextPage() );
}

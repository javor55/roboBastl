# Arduino 

## Úvod do Arduino světa
### Co je to arduino?

Wikipedie říká: 
> Arduino [čti Arduíno] je název malého jednodeskového počítače založeného na mikrokontrolerech ATmega od firmy Atmel. Svým návrhem se snaží podpořit výuku informatiky ve školách a seznámit studenty s tím, jak jsou pomocí počítačů řízena různá zařízení (např. mikrovlnná trouba, automatická pračka a jiné stroje). Nejedná se tedy o počítač ve smyslu stolního počítače nebo chytrého telefonu. Nelze proto k němu snadno přímo připojit monitor ani klávesnici či myš, ale je připraven na připojení LED diod, LCD displeje , servomotorů, senzorů, osvětlení atd.

- open source vývojová platforma původem z Itálie ([20 různých desek][5]), mnoho dalších variant a shieldů ([odkaz][6]) 
- postavená okolo procesoru ATMega
- programovací jazyk [Wiring][1] (takové C++)
- vývojové prostředí [Arduino][2] (jen trochu chytřejší textový editor), [Platformio][3] nebo programování ve [Visual Studiu][4]

[1]: https://arduino.cz/zakladni-struktury-jazyka-wiring/
[2]: https://www.arduino.cc/en/Main/Software
[3]: https://platformio.org/
[4]: https://marketplace.visualstudio.com/items?itemName=VisualMicro.ArduinoIDEforVisualStudio
[5]: https://www.arduino.cc/en/Products/Compare
[6]: http://playground.arduino.cc/Main/SimilarBoards

### Bližší pohled na desku
- V našem případě budeme používat desku Arduino UNO
- porty: 
    - 14 digitálních vstupů nebo výstupů (záleží na konkrétním nastavení) číslovaných 0 až 13
    - šest analogových vstupů (0–6)
    - různé vývody napájení (napájecí napětí, 5V stabilizovaných, 3.3V, GND)

![ArduinoUno](./Images/UNOV3PDF.png)

### Programování 
#### Programovací prostředí (IDE)
Není tak komplikovaný jako "vyspělá" prostředí pro pokročilé jazyky. V hlavní časti okna nalezneme prostor pro náš kód, nad ním několik užitečných tlačítek pro zkontrolování kódu, nahrávání programu, uložení, atd. V řádků navigačních prvků nás bude zajímat nabídka Nástroje, kde musíme nastavit správnou desku a seriový port.
<<<<<<< HEAD
![ArduinoIDE](./Images/ArduinoIDE.png)
=======
![ArduinoIDE](./ArduinoIDE.PNG)
>>>>>>> 2bb43a1fa9f8908fc483034817a029e72ecb724b
#### Struktura programu
Základní struktura programovacího jazyka Arduino je poměrně jednoduchá a skládá se nejméně ze dvou
částí, přesněji funkcí.

Funkce `setup()` by měla být volána až po deklaraci všech proměnných na začátku programu. Tato funkce se
používá například k nastavení pinů Arduina na vstup nebo výstup, nastavení parametrů sériové komunikace
a podobných jednorázových akcí.

Po funkci `setup()` následuje funkce `loop()`. Tělo této funkce obsahuje programový kód, který bude opakovaně
prováděn v nekonečné smyčce, například čtení vstupů, nastavování výstupů, výpočty, atd. Tato funkce je
jádrem všech programů Arduina a vykonává většinu činností. 

```cpp
int ledPin = 13; // pojmenování LED

void setup () {
 pinMode(ledPin, OUTPUT); // nastavení pinu do digitální výstup
}

void loop () {
 digitalWrite(ledPin, HIGH); // zapnutí LED
 delay (1000); // počkáme jednu sekundu (1000 milisekund)
 digitalWrite(ledPin, LOW); // vypnutí LED
 delay (1000); // počkáme jednu sekundu 
}
```

**podmínky**
```cpp
if (temp >= 22)
{
  //Vypnout topení
}
else if (temp >= 18 && temp < 22)
{
  //Nedělej nic
}
else
{
  //Zapnout topení
}
```
**cykly**

```cpp
for (int i=0; i <= 255; i++){
      print(i);
      delay(10);
   }
```
```cpp
var = 0;
while(var < 200){
  // do something repetitive 200 times
  var++;
}
```


Detailnější úvod do programování naleznete [ZDE](http://www.hobbyrobot.cz/wp-content/uploads/ArduinoPriruckaProgramatora.pdf)

## Příklady

### Blíkání LED a používání tlačítek
_logická úroveň, zpoždění, digitální výstup, LED diody, analog write_

```cpp
int ledPin = 13; // pojmenování LED

void setup () {
 pinMode(ledPin, OUTPUT); // nastavení pinu do digitální výstup
}

void loop () {
 digitalWrite(ledPin, HIGH); // zapnutí LED
 delay (1000); // počkáme jednu sekundu (1000 milisekund)
 digitalWrite(ledPin, LOW); // vypnutí LED
 delay (1000); // počkáme jednu sekundu 
}
```
#### Modifikace 0,5
Zpoždění bez příkazu `delay()`
 ```cpp
// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin

// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)

void setup() {
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // here is where you'd put code that needs to be running all the time.

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}
```
#### Modifikace 0,7
Zpoždění bez příkazu `delay()` a schvonání kódu do funkce
 ```cpp
unsigned long previousMillis = 0;      
int ledPin = 13;
int ledState = 0;

void setup() {
 // set the digital pin as output:
 pinMode(ledPin, OUTPUT);
}

void loop() { 
 if (waitMoment(1000)) {    
   if (ledState == LOW) {
     ledState = HIGH;
   } else {
     ledState = LOW;
   }
   // set the LED with the ledState of the variable:
   digitalWrite(ledPin, ledState);
 }
}

bool waitMoment(int interval) {
   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
       previousMillis = currentMillis;
       return true;
   }
   return false;    
}
```

#### Modifikace 1
připojíme LED přes nepájívé pole

rezistor 220 Ohm
![ArduinoIDE](./Images/blink.png)
#### Modifikace 2
Vytvoření blikačky

#### Modifikace 3 
Postupné zesilování a zeslabování ledky

- Analogový výstup `analogWrite(pin, value)` (0 - 255) 

```cpp
int ledPin = 9; // pojmenování LED
int value = 0;    // pocatecni rozsviceni
int step = 5;    // jak velký bude krok 

void setup () {
 pinMode(ledPin, OUTPUT); // nastavení pinu do digitální výstup
}

void loop () {
    analogWrite(ledPin, value);
 
    value = value + step;

    // zmenim smer kdyz jsem na konci
    if (value <= 0 || value >= 255) {
        step = step*-1;
    }
    delay(20);
}
```
#### Modifikace 4 
Připojení tlačítka

rezostor 10K Ohm
![ArduinoIDE](./Images/button.png)
```cpp
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
}
```

varianta s připojením input-pullup rezistoru softwarově 
![ArduinoIDE](./Images/button_pullup.png)
```cpp
void setup() {
  //start serial connection
  Serial.begin(9600);
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);

}

void loop() {
  //read the pushbutton value into a variable
  int sensorVal = digitalRead(2);
  //print out the value of the pushbutton
  Serial.println(sensorVal);

  // Keep in mind the pull-up means the pushbutton's logic is inverted. It goes
  // HIGH when it's open, and LOW when it's pressed. Turn on pin 13 when the
  // button's pressed, and off when it's not:
  if (sensorVal == HIGH) {
    digitalWrite(13, LOW);
  } else {
    digitalWrite(13, HIGH);
  }
}
```
#### Modifikace 5
Detekce hran na tlačítku 
```cpp
// this constant won't change:
const int  buttonPin = 2;    // the pin that the pushbutton is attached to
const int ledPin = 13;       // the pin that the LED is attached to

// Variables will change:
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
bool state = 0;

void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);  
}


void loop() {
  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    
    if (buttonState == HIGH) {
      state = true;
    } else {
      state = false;
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  
  digitalWrite(ledPin, state);  
}
```

### Seriová komunikace
Slouží pro komunikaci s PC, případně Bluetooth. 
```cpp
void setup() {                
// Turn the Serial Protocol 
  Serial.begin(9600);
}
```
otevření seriové komunikace
```cpp
void setup() {                
// Turn the Serial Protocol 
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
}
```
Posílání dat přes seriovou linku probíha pomocí příkazů `Serial.print()` a `Serial.println()`, 
čtení dat pomocí `Serial.available()` a `Serial.read()`.

Příklad:
```cpp
void setup() {                
// Turn the Serial Protocol 
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
}
void loop(){
    Serial.print("Máma mele maso");
    delay(500);
    
    while (Serial.available()>0){
        serIn = Serial.read();	//read Serial        
        Serial.print(serIn, BYTE); 	//prints the character just read
     }
}
```
### Používání Multi-Functional shieldu

<<<<<<< HEAD
### Co dál? 
  - ukázat zakladni měřící přístroje 
  - vymyslet co delat... 
=======
https://www.mpja.com/download/hackatronics-arduino-multi-function-shield.pdf


## Kdyby vás to zaujalo

### Koho sledovat na youtube
[GreatScott!](https://www.youtube.com/user/greatscottlab)

[educ8s.tv!](https://www.youtube.com/channel/UCxqx59koIGfGRRGeEm5qzjQ)

[MKme Lab](https://www.youtube.com/channel/UCTo55-kBvyy5Y1X_DTgrTOQ)

[Mert Arduino and Tech](https://www.youtube.com/channel/UCAH7OfjndkAgtjkmJ6IQvEw/videos)


### A ideální košík na Aliexpresu

- [Arduino Nano][A1]
- [Nepájivé pole][A2]
- [Nějaké odpory, diody, tlačítka a potenciometry][A3]
- Nějaký displej ( [1602][A6], [0,96" OLED][A7], [Nokia 5110 displej][A8] ) 
- [Nějaký sensor kit][A5]
- [Nebo kompletní Arduino Starter Kit][A4]

[A1]: https://www.aliexpress.com/item/Freeshipping-Nano-3-0-controller-compatible-for-arduino-nano-CH340-USB-driver-NO-CABLE/32341832857.html?spm=2114.search0104.3.1.31162b20Kwform&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142-10621,searchweb201603_38,ppcSwitch_5&algo_expid=aaf6c13f-3a9b-475b-88f9-9143cf7ef819-0&algo_pvid=aaf6c13f-3a9b-475b-88f9-9143cf7ef819&priceBeautifyAB=0
[A2]: https://www.aliexpress.com/item/MB102-Breadboard-power-module-MB-102-830-points-Solderless-Prototype-Bread-board-kit-65-Flexible-jumper/32690555189.html?spm=2114.search0104.3.1.4fbc3e446f2PLl&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142-10621,searchweb201603_38,ppcSwitch_5&algo_expid=e8b55d90-b5cc-4864-bc58-2ecf5f5be56f-0&algo_pvid=e8b55d90-b5cc-4864-bc58-2ecf5f5be56f&priceBeautifyAB=0
[A3]: https://www.aliexpress.com/item/HX-Studio-Electronics-component-pack-with-resistors-LEDs-Switch-Potentiometer-for-Arduino-UNO-MEGA2560-Raspberry-Pi/32811565832.html?spm=2114.search0104.3.8.3d1c57a9JeEbUx&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142,searchweb201603_38,ppcSwitch_5&algo_expid=79b6b1c6-14f5-4d46-a3c9-303b2be6e0c0-1&algo_pvid=79b6b1c6-14f5-4d46-a3c9-303b2be6e0c0&priceBeautifyAB=0
[A4]: https://www.aliexpress.com/item/NEWEST-RFID-Starter-Kit-for-Arduino-UNO-R3-Upgraded-version-Learning-Suite-With-Retail-Box/32714696336.html?spm=2114.search0104.3.1.32631e47P9xXQN&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142-10621,searchweb201603_38,ppcSwitch_5&algo_expid=1c223a8e-ce64-4dc3-8435-72a0d1553b19-0&algo_pvid=1c223a8e-ce64-4dc3-8435-72a0d1553b19&priceBeautifyAB=0
[A5]: https://www.aliexpress.com/item/37-in-1-box-Sensor-Kit-For-Arduino-Starters-brand-in-stock/32695281182.html?spm=2114.search0104.3.2.3481cbf4s2oWgS&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142,searchweb201603_38,ppcSwitch_5&algo_expid=54f79b86-3fcd-417e-948d-90a171a8594e-0&algo_pvid=54f79b86-3fcd-417e-948d-90a171a8594e&priceBeautifyAB=0
[A6]: https://www.aliexpress.com/item/1PCS-LCD-module-Blue-screen-IIC-I2C-1602-for-arduino-1602-LCD-UNO-r3-mega2560/32763867041.html?spm=2114.search0104.3.2.69971844dw50if&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142,searchweb201603_38,ppcSwitch_5&algo_expid=5ca41b5c-0110-4324-9854-5a6a38c4e40f-0&algo_pvid=5ca41b5c-0110-4324-9854-5a6a38c4e40f&priceBeautifyAB=0
[A7]: https://www.aliexpress.com/item/0-96-inch-IIC-Serial-White-OLED-Display-Module-128X64-I2C-SSD1306-12864-LCD-Screen-Board/32780054633.html?spm=2114.search0104.3.44.56a94ddewbE0Nd&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142,searchweb201603_38,ppcSwitch_5&algo_expid=575927fb-2ddd-47c0-8028-5a0dd1369e6f-6&algo_pvid=575927fb-2ddd-47c0-8028-5a0dd1369e6f&priceBeautifyAB=0
[A8]: https://www.aliexpress.com/item/84-48-LCD-Module-White-backlight-adapter-pcb-for-Nokia-5110/1859113549.html?spm=2114.search0104.3.2.62f21848YXZV3Q&ws_ab_test=searchweb0_0,searchweb201602_2_10152_10151_10065_10344_10068_10342_10343_10340_10341_10084_10083_10618_10304_10307_10302_5711211_5722315_10313_10059_10534_100031_10629_10103_10626_10625_10624_10623_10622_10621_10620_10142-10620,searchweb201603_38,ppcSwitch_5&algo_expid=8803482e-708e-4cdb-91fc-559c0519cbd3-0&algo_pvid=8803482e-708e-4cdb-91fc-559c0519cbd3&priceBeautifyAB=0


>>>>>>> 2bb43a1fa9f8908fc483034817a029e72ecb724b

## Zdroje:

[ARDUINO – příručka programátora ](http://robotikabrno.cz/docs/arduino/Pr%C5%AFvodce-sv%C4%9Btem-Arduina-CZ.pdf)

[Průvodce světem Arduina](http://robotikabrno.cz/docs/arduino/Pr%C5%AFvodce-sv%C4%9Btem-Arduina-CZ.pdf)

[Arduino: jak pro něj začít programovat](https://www.root.cz/clanky/arduino-jak-pro-nej-zacit-programovat)

[itnetwork.cz](https://www.itnetwork.cz/hardware-pc/arduino)

[Wikipedia](https://cs.wikipedia.org/wiki/Arduino)

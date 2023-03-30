//Kode for multimeter til arbeidsoppdrag 1, Måleteknikk.
//Kode laget av; Tarje, Tobias, Andreas.

//deskriptiv tekst er skrevet på hvor det har vært nødvendig å forklare en kode. Derav hvor koder går igjen er dette ikke forklart om igjen.


//____________________________________________InnholdsFortegnelse__________________________________________________________________________//
//|                       Innholdsfortegnelsen tar utgangspunkt i linjenummer:                                                             |//
//|                                                                                                                                        |//
//|                  Linje; 1-146...................................................................Defineringer & inkludere biblioteker   |//
//|                  Linje; 147-262.................................................................Void Setup                             |//
//|                  Linje; 263-278.................................................................Void Loop                              |//
//|                  Linje; 279-1208................................................................Funksjoner                             |//
//|                  Linje; 282-383.................................................................Oled skjerm funksjon                   |//
//|                  Linje; 386-390.................................................................Portmeter funksjon                     |//
//|                  Linje; 393-421.................................................................DC-spenning funksjon                   |//
//|                  Linje; 423-471.................................................................AC-strøm                               |//
//|                  Linje; 473-505.................................................................AC-spenning                            |//
//|                  Linje; 507-992.................................................................Resistansmåling                        |//
//|                  Linje; 995-1027................................................................Diodemåling                            |//
//|                  Linje; 1030-1084...............................................................kontinuitet                            |//
//|                  Linje; 1087-1144...............................................................Kapasitans                             |//
//|                  Linje; 1146-1172...............................................................Induktans                              |//
//|                  Linje; 1175-1184...............................................................Temperatur                             |//
//|                  Linje; 1186-1208...............................................................Batterikapasitet                       |//
//|________________________________________________________________________________________________________________________________________|//



//////////////////////////////////////////////////**Defineringer**///////////////////////////////////////////////////////////
//________________________________________Inkluderte biblioteker_________________________________________________//
#include <SPI.h>
# include <Wire.h>
# include <Adafruit_GFX.h>//                                      Bibliotek brukt til oled skjerm
# include <Adafruit_SSD1306.h>//                                      Bibliotek brukt til oled skjerm
//________________________________________Oled-skjerm defineringer_________________________________________________//
# define SCREEN_WIDTH 128 //                                            Definere oled bredde                                              
# define SCREEN_HEIGHT 64 //                                              definere oled høyde
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);//              henter funksjon fra adafruit_1306 og gir dermed oled skjermen navnet "skjerm", henter deretter bredde og høyde fra definert over.
//                                                                              bruker -1 for å si at reset pinnen på oled skjermen ikke er i bruk.
//________________________________________Oppstartskjerm variabler_________________________________________________//
  int bredde = 100;//                                   Setter bredde på loading baren til oled skjermen
  int hoyde = 10;//                                   Setter høyde på loading baren til oled skjermen
  int barStart;//                                   Setter opp variabel for hvor loading baren skal starte
  int barSlutt;//                                   Setter opp variabel for hvor loading baren skal slutte

  int loadingBredde = 0;//                                Setter opp bredden på loadingen
  int loadingHoyde = 0;//                               Setter opp setter opp høyde på loadingen
  int loadingStart;//                               Setter opp hvor insiden av den oppfylte baren skal starte
  int loadingSlutt;//                               Setter opp hvor insiden av den oppfylte baren skal slutte

  int startTidLoading;                          //setter opp variabler for telling av tid
  int tid;                          //setter opp variabler for telling av tid

  float stopp;
//________________________________________portMeter variabler_________________________________________________//
int portVerdi;
unsigned int portTeller;
unsigned int portGjennomsnitt;
//________________________________________resistansMåling variabler_________________________________________________//
int res1=27;//        100ohm                            Referansemotstand brukt i kretsen for måling av resistans
int res2=26;//        1kohm                        Referansemotstand brukt i kretsen for måling av resistans
int res3=25;//        10kohm                            Referansemotstand brukt i kretsen for måling av resistans
int res4=33;//        100kohm                           Referansemotstand brukt i kretsen for måling av resistans

bool kilo=false;//                                        setter variabel for når resistansen målt er kilo eller ikke

int resistans1=100;         //mulig må endres på for kalibrering av resistansmåler
int resistans2=1000;          //mulig må endres på for kalibrering av resistansmåler
int resistans3=10000;         //mulig må endres på for kalibrering av resistansmåler
int resistans4=100000;          //mulig må endres på for kalibrering av resistansmåler
int valgtResistans=0;
float kalibreringR=1.07;

float ohmMaling, utregningOhm, utregningOhm2;//                     Variabler brukt til utregning av ohm
float r1, r2, r3;//                                         brukt under testing, ikke i brukt

//________________________________________Diode variabler_________________________________________________//
int diodeMaling;
float utregnetDiode;
//________________________________________Buzzer variabler_________________________________________________//
int buzzer=2;

//________________________________________Spenningskilde_________________________________________________//
float spenningKilde=3.30;//                                 definerer spenningskilde for utregning av dc spenning

//________________________________________Spenning variabler_________________________________________________//
      float spenningRes1 = 10000.00; 
      float spenningRes2 = 4700.00; 
      float espSpenning = 3.30; 
      float vdrRes = 0.00; 
      float malingSpenning = 0.00; 
      float volt = 0.00; 
      float spenning=0.00;

      uint32_t v;

//________________________________________temperatur variabler_________________________________________________//
int resistansTemp = 10000;
float logRes;
float resistansVerdi;
float temperatur;
float temp;

float therm1=1.104965971e-03;//                          Coefficient verdier til en 10k thermistor. Kalkulert via coefficient kalkulator på 5grader, 25grader og 40grader.
float therm2=2.387549993e-04;
float therm3=0.6404017973e-07;

//________________________________________Induktanse variabler_________________________________________________//
double pulse, frekvens, kapasitans, induktanse, induktanse_mH;              //brukt under testing av induktans, måling av induktans fungerer ikke optimalt ved bruk av ESPen ettersom det ikke var nok tid til feilsøking
const int freq = 20000;                                                   //brukt under testing av induktans, måling av induktans fungerer ikke optimalt ved bruk av ESPen ettersom det ikke var nok tid til feilsøking
volatile int ledChannel = 0;                                                    //brukt under testing av induktans, måling av induktans fungerer ikke optimalt ved bruk av ESPen ettersom det ikke var nok tid til feilsøking
const int resolution = 8;                                                   //brukt under testing av induktans, måling av induktans fungerer ikke optimalt ved bruk av ESPen ettersom det ikke var nok tid til feilsøking



//________________________________________Kontinuitet variabler_________________________________________________//
  int outerRadius = SCREEN_WIDTH / 8;//                                        Setter insideradius til sirkelen laget for måling av kontakt.
  int centerX = SCREEN_WIDTH / 2;//                                        Setter bredde på sirkelen
  int centerY = (SCREEN_HEIGHT / 2)+10;//                                        Setter høyde på sirkelen
  int innerRadius = outerRadius / 2;//                                        Setter størrelse på sirkelen

//________________________________________Batteriprosent_________________________________________________//
    float batteriProsent;

//________________________________________Kapasitans_________________________________________________//
        double C; 
        bool mikro=true;
//________________________________________Strøm måler_________________________________________________//
const int sensorIn = 15;      // pin where the OUT pin from sensor is connected on Arduino
int mVperAmp = 100;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
float result;

//________________________________________batteri & temp til oled_________________________________________________//
int oledProsent;
int oledTemp;





void setup() 
{ 
   Serial.begin(115200);//  
  // induktanse//
ledcSetup(ledChannel, freq, resolution);      //            brukt til testing av induktans
ledcAttachPin(14, ledChannel);      //            brukt til testing av induktans

  pinMode(buzzer,OUTPUT);//                            setter buzzerpin til output
  pinMode(34, INPUT);                                    //portmeter pin som input for analog

//___________________________________________Strøm/effekt___________________________________________//
  Serial.println ("ACS712 current sensor"); 

//----------------------------------------**OLED skjerm startup**-------------------------------------------------------//
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.display();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(10, 40);
  oled.print("Loading");
                                       
  barStart=(SCREEN_WIDTH - bredde) / 2;//                                     Setter verdi på hvor progresjonsbaren skal starte
  barSlutt=(SCREEN_HEIGHT - hoyde) / 2;//                                    Setter verdi på hvor progresjonsbaren skal slutte
  oled.drawRect(barStart, barSlutt, bredde, hoyde, WHITE);//                Funksjon for å lage et rektangel med lagde verdier
//                                                                     Setter verdier på variabler som brukes til å fylle progresjonsbaren
  loadingHoyde= hoyde-2;//                                           setter høyde på loading bar                                            
  loadingStart= barStart+1;//                                           Setter startpunkt på loadingbar
  loadingSlutt= barSlutt+1;//                                           setter sluttpunkt på loadingbar
  startTidLoading = millis();//                                           lager variabel for å starte å telle millisekunder
  while (loadingBredde < (bredde - 2)/2)//                       While loop for å gradvis fylle opp progresjonsbaren, deler på to for å bare få progresjonsbaren til 50%
   {
    tid = millis() - startTidLoading;//                                  setter tid til å være lik opptelte millisekunder minus starttid         
    stopp = (float)tid / 2000;//                            Deler på 2000 for tiden det skal ta å fylle progresjonsbaren
      if (stopp > 1.0) //                            stopper while loopen dersom tiden overstiger 2sekunder
      {
        stopp = 1.0;//                            Setter stopp til 1 når while loopen skal stoppe for å alltid stoppe på samme sted
      }
    loadingBredde = stopp * (bredde - 2);//                       Regner ut hvor mye som er fyllt opp av progresjonsbaren
    oled.fillRect(loadingStart, loadingSlutt, loadingBredde, loadingHoyde, WHITE);//           Funksjon for å fylle opp rektangelet
    oled.display();//                            Sende ut verdier til oled skjerm
  }
//                                              intensjonell blinkende "critical error"
  delay(1000);
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(20, 15);
  oled.print("Critical");
  oled.setCursor(35, 35);
  oled.print("ERROR"); 
  oled.display();
  delay(500);
    oled.clearDisplay();
    oled.display();
    delay(500);
      oled.setTextSize(2);
      oled.setTextColor(WHITE);
      oled.setCursor(20, 15);
      oled.print("Critical");
      oled.setCursor(35, 35);
      oled.print("ERROR"); 
      oled.display();
      delay(500);
        oled.clearDisplay();
        oled.display();
        delay(500);
          oled.setTextSize(2);
          oled.setTextColor(WHITE);
          oled.setCursor(20, 15);
          oled.print("Critical");
          oled.setCursor(35, 35);
          oled.print("ERROR"); 
          oled.display();
          delay(500);
            oled.clearDisplay();
            oled.display();
            delay(500);
//                                   Lager progresjonsbar for "bypassing" av "error" joke
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(10, 40);
  oled.print("Bypassing");

  barStart=(SCREEN_WIDTH - bredde) / 2;
  barSlutt=(SCREEN_HEIGHT - hoyde) / 2;
  oled.drawRect(barStart, barSlutt, bredde, hoyde, WHITE);

//                        Resette variabler for progresjonsbar
  loadingHoyde= hoyde-2;
  loadingStart= barStart+1;
  loadingSlutt= barSlutt+1;
  stopp=0;
  loadingBredde= 0;
  startTidLoading=millis()-startTidLoading;
  
  while (loadingBredde < (bredde - 2))//                                   deler ikke på to her for å fylle progresjonsbaren til 100%
   {
//                                        Calculate the current progress
    tid = millis() - startTidLoading;
    stopp = (float)tid / 2000;
    if (stopp > 1.0) 
    {
      stopp = 1.0;//                            
    }
//                                         Calculate the width of the filled portion of the progress bar
    loadingBredde = stopp * (bredde - 2);
//                                                                Draw the filled portion of the progress bar
    oled.fillRect(loadingStart, loadingSlutt, loadingBredde, loadingHoyde, WHITE);
    oled.display();//                                   Sender ut verdier til oled skjerm
  }
  delay(1000);//                                   delay for å stanse koden i et sekund etter baren har fyllt seg opp
}

//////////////////////////////////////////**LOOP**////////////////////////////////////////////////////////////////////////
void loop() 
{
  rotasjon();//                                   Rotasjonfunksjon for portmeteret. Her vil koden lese av verdien som kommer inn fra portmeteret og dele det opp i antallet målinger.
  batteriKapasitet();//                                 funksjon for måling av batterikapasitet
  resistansMaling();//                                   Funksjon for måling av resistans på målepinnene. 
  diodeMaling1();//                                         Funksjon for måling av spenning over dioder
  spenningsMaler();//                                         funksjon for måling av dc spenning 
  kontinuitetsMaling();//                                   Funksjon for måling av kontakt (kontinuitet ble ikke tatt med)
  temperaturMaling();//                                   Funksjon for måling av temperatur
  induktansMaling();//                                   Funksjon for måling av induktans
  acSpenning();//                                   Funksjon for måling av ac spenning
  stromMaler();//                                 Funksjon for måling av Strøm
  kapasitansMaling();//                                 Funksjon for måling av kapasitans
  oledSkjerm();//                                   funksjon for display av hver enkelt funksjon til oled displayet
  delay(1000);//                                   stopper koden i et sekund etter hver runde i loopen
}
//////////////////////////////////////////**Funksjoner**///////////////////////////////////////////////////////////////////

//////////////////////////////////////////**OLED**/////////////////////////////////////////////////////////////////////////
  void oledSkjerm()//                                   Funksjon for oledskjerm
  {
///________________________________________DC Spenning_____________________________________________________///                                   Utskriving av spenningsfunksjon til oled skjerm
      if (portVerdi==1)//                                   if løkke for hver enkel verdi av utmappingen av portmeteret for å velge hvilken funksjon som skal skrives ut
    {
      //spenningsmåling

      oled.clearDisplay();
      oled.setTextSize(2),oled.setTextColor(WHITE),oled.setCursor(30, 40),oled.println(String(spenning)+String("V"));; //samler oled koder på en linje for å komprimere koden
      oled.setTextSize(1),oled.setCursor(25, 20),oled.println("DC Spenning");
      //temp og batt
      oled.setCursor(5, 5),oled.println(String(oledProsent)+String("%")),oled.setCursor(100, 5),oled.println(String(oledTemp)+String("C"));//         Setter på verdier, plasseringer , størrelse, farge
      oled.display(); //                      Printer innholdet ut på oled skjermen
    }
///_______________________________________AC spenning_____________________________________________________///
    if (portVerdi==2)
    {
        //ac spenning:

        oled.clearDisplay(),oled.setTextSize(2),oled.setTextColor(WHITE),oled.setCursor(30, 40), oled.println(String(v)+String("V"));
      oled.setTextSize(1), oled.setCursor(25, 20), oled.println("AC Spenning");
            //temp og batt
      oled.setTextSize(1), oled.setTextColor(WHITE),oled.setCursor(5, 5), oled.println(String(oledProsent)+String("%"));
      oled.setCursor(100, 5),oled.println(String(oledTemp)+String("C"));
      oled.display(); 
    }
///_______________________________________Strøm/effekt_____________________________________________________///
    if (portVerdi==3)
    {
      //strøm&effekt:
      oled.clearDisplay();
      oled.setTextSize(1), oled.setTextColor(WHITE),oled.setCursor(30, 35),oled.println(String(AmpsRMS)+String("A")),oled.setCursor(30, 45),oled.println(String(Watt)+String("W"));
      oled.setTextSize(1), oled.setCursor(25, 20),oled.println("Strom/effekt");
                  //temp og batt
      oled.setCursor(5, 5), oled.println(String(oledProsent)+String("%")),oled.setCursor(100, 5), oled.println(String(oledTemp)+String("C"));
      oled.display(); 
    }
///_______________________________________Diode_____________________________________________________///
    if (portVerdi==4)
    {
      //diodemåling:
        oled.clearDisplay();
        oled.setTextSize(2), oled.setTextColor(WHITE),oled.setCursor(30, 40), oled.println(String(utregnetDiode)+String("V"));
        oled.setTextSize(1), oled.setCursor(25, 20), oled.println("Diode");
      //batteri og temp
      oled.setCursor(5, 5),oled.println(String(oledProsent)+String("%")),oled.setCursor(100, 5), oled.println(String(oledTemp)+String("C"));
      oled.display(); 
    }
///_______________________________________Resistans_____________________________________________________///
    if (portVerdi==5)
    {
      //resistansmåling:

      oled.clearDisplay();
      oled.setTextSize(2),oled.setTextColor(WHITE), oled.setCursor(20, 40),oled.println(String(utregningOhm));
      oled.setTextSize(2),oled.setCursor(110, 40);
      if (kilo==true){ //                                                       Setter på if løkke for å skrive ut verdien i kiloOhm
      oled.println("k");
      }
      if(kilo==false){  //                                                  Setter if løkke på å skrive ut verdien om den ikke er i kiloOhm
        oled.println("");
      }
      oled.setTextSize(1), oled.setCursor(25, 20),oled.println("Resistans");
                  //temp og batt
      oled.setCursor(5, 5), oled.println(String(oledProsent)+String("%")),oled.setCursor(100, 5), oled.println(String(oledTemp)+String("C"));
      oled.display(); 
      
    }
///___________________________________________Kapasitans___________________________________________///
    if (portVerdi==6)
    {
      //kapasitansmåling:

      oled.clearDisplay();
      oled.setTextSize(2), oled.setTextColor(WHITE),oled.setCursor(30, 40); 
      if (mikro==true){
      oled.println(String(C)+String("uF"));
      }
      if(mikro==false){
        oled.println(String(C*1000)+String("nF"));
      }
      oled.setTextSize(1), oled.setCursor(25, 20), oled.println("Kapasitans");
                        //temp og batt
      oled.setCursor(5, 5), oled.println(String(oledProsent)+String("%")),oled.setCursor(100, 5), oled.println(String(oledTemp)+String("C"));
      oled.display(); 
    }

  
///___________________________________________INDUKTANS___________________________________________///
if (portVerdi==7)
{
        oled.clearDisplay();
        oled.setTextSize(2), oled.setTextColor(WHITE),oled.setCursor(30, 40), oled.println(String(induktanse)+String("H"));
        oled.setTextSize(1), oled.setCursor(25, 20), oled.println("induktanse");
      //batteri og temp
      oled.setCursor(5, 5),oled.println(String(oledProsent)+String("%")),oled.setCursor(100, 5), oled.println(String(oledTemp)+String("C"));
      oled.display(); 
}
//internt i induktans kode
///_______________________________________Buzzer/kontakt_____________________________________________________///
//internt i kontakt kode
}
//////////////////////////////////////////**ROTASJON**/////////////////////////////////////////////////////////////////////////

  void rotasjon()//                                   rotasjonsfunksjon hvor verdien innhentet fra portmeteret er mappet ut fra 0-4095 som er hvor mye espen leser av til 0-12 som er antallet målinger
  {
        portVerdi=analogRead(34);//                             Leser av verdien fra potensiometeret
        portVerdi=map(portVerdi,0,4095,8,0);  //                    Deler verdien fått fra potensiometeret for antallet målingsfunksjoner som er 8
  }

//////////////////////////////////////////**DC Spenning**/////////////////////////////////////////////////////////////////////////
void spenningsMaler() //                                   Funksjon for dc spenningsmåling
{ 
  while (portVerdi==1)
  {
    pinMode(32,INPUT);
        rotasjon();//                                   Leser av portmeterverdien for å bestemme når while loopen skal stoppes

      vdrRes = (spenningRes2 / (spenningRes1 + spenningRes2)); 
      for (int i = 0; i < 20 ; i++)                                   //må skjekkes, kan være denne skal være 21 om første gang den går gjennom ikke plusser på 1
      { 
        malingSpenning = malingSpenning + analogRead(32); 
        delay(3); 
      } 
      malingSpenning = malingSpenning / 21; 
      volt = ((malingSpenning * espSpenning) / 4095);       
      spenning = volt / vdrRes; 
      if (spenning>0.05)//                                plusser på spenningstap over diode og tyristor (ekstra kalibrering trengs etter lodding av komponenter)
      {
        spenning= spenning+1.50;

      }
//                                                    oled skjerm funkskjon må settes inn i koden for spenningsmåling for å oppdattere tallene (bør undersøkes om det blir nok tid)
      oledSkjerm();
      batteriKapasitet();
      temperaturMaling();
      delay(1000);
      rotasjon();//                                   Leser av portmeterverdien for å bestemme når while loopen skal stoppes
  }         
} 
//////////////////////////////////////////**AC Strøm**/////////////////////////////////////////////////////////////////////////
void stromMaler(){
 while(portVerdi==3)
 {
   rotasjon();
  Serial.println (""); 
  Voltage = getVPP();
  VRMS = (Voltage/2.0) *0.707; 
  AmpsRMS = ((VRMS * 1000)/mVperAmp)-0.15;// utregner amper
 
  Serial.print(AmpsRMS);
  Serial.print(" Amps RMS  ---  ");
  Watt = (AmpsRMS*230/1); //              benytter 230V som referanse ved utregning av watt (her måtte spenningskilden endres til 9volt om effekt skulle regnes ut over dc batteri på 9v)
  Serial.print(Watt);
  Serial.println(" Watts");
 
   oledSkjerm();
  batteriKapasitet();
  temperaturMaling();
  delay(1500);
  
 }
}
float getVPP()
{
   if(portVerdi==3)
 {

  int readValue;                //Variabel for avelsning av sensoren
  int maxValue = 0;             //Lagrer max verdi på maxValue
  int minValue = 4096;          // Lagrer minimumsverdi
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(sensorIn);
       if (readValue > maxValue) 
       {
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           minValue = readValue;
       }
   }
   result = ((maxValue - minValue) * 3.3)/4096.0; 
      
   return result;
 }
}
//////////////////////////////////////////**AC spenning**/////////////////////////////////////////////////////////////////////////
uint16_t get_max() 
{
  if (portVerdi==2)
  {
    uint16_t max_v = 0;
    for(uint8_t i = 0; i < 100; i++) {
      uint16_t r = analogRead(32);
      if(max_v < r) max_v = r;
      delayMicroseconds(200);
    }
    return max_v;
  }
}
 void acSpenning()
 {
    while(portVerdi==2)
    {
    rotasjon();
    get_max();
      char buf[10];
       v = get_max();
      v = v * 1019/4095;//      benytter 1,019V som referansespenning ettersom dette er utregnet verdi over spenningsdeleren i kretsen
      v /= sqrt(2);//             Delere på roten av to for å finne spenningen ettersom kretsen nå kun tar med seg toppverdien på sinuskurven.

      sprintf(buf, "%03u Volts", v);
      oledSkjerm();
      batteriKapasitet();
      temperaturMaling();
      Serial.println(String("volt = ")+String(v));
      oledSkjerm();
      delay(1000);
  }
 }
//////////////////////////////////////////**RESISTANS**/////////////////////////////////////////////////////////////////////////
void resistansMaling()//                                   funksjon for måling av resistans
{ 
      while (portVerdi==5)                              //resistansmåling er kalibrert fra 22ohm til 100k ohm og alle målinger utenfor dette området har blitt kuttet. Dette er grunnet mangel på resistorer til å kalibrere.
      {
        kilo=false;
        float spenningKilde=3.3;//                                   definerer spenningskilden som 3.3v som er hva espen sender ut
        valgtResistans=0;//                                   Setter en variabel for om koden har valgt en referansemotstand
          rotasjon();//                                   Rotasjonsfunksjon for å si ifra når while loopen skal stoppes
          pinMode(res1,OUTPUT);//                                   Setter første referansemotstand som output pin og alle andre pins som en input for å ikke påvirke resultatet
          pinMode(res2,INPUT);
          pinMode(res3,INPUT);
          pinMode(res4,INPUT);
          digitalWrite(res1,HIGH);//                                   Sender ut spenning på første referansemotstand pin og setter alle andre til lav for å ikke påvirke resultatet
          digitalWrite(res2,LOW);
          digitalWrite(res3,LOW);
          digitalWrite(res4,LOW);
          ohmMaling=analogRead(35);
          Serial.println(String("ref1 ")+String(ohmMaling));//                                   Brukt under testing. Kan SLETTES

            if (ohmMaling>1150&&ohmMaling<3500)//                                   valgt avlesning for området hvor koden skal velge 100ohm som referansemotstand
            {
              if (valgtResistans==0)//                                   Dersom ingen andre referansemotstander har blitt valgt kan koden velge en referansemotstand. 
              {//                                                         (brukt under utvikling, men ettersom dette er første løkke vil ikke denne if løkken ha noe å si siden "valgtresistans" alltid vil være 0)
                valgtResistans=1;
                //utregningOhm=(((spenningKilde/4095)*ohmMaling)*resistans1)/(spenningKilde-((spenningKilde/4095)*ohmMaling));//  Formel for utregning av motstand med brukt referansemotstand og kildespenning
                  
                  for (int i = 0; i < 20 ; i++)                                   //
                  { 
                    ohmMaling = ohmMaling + analogRead(35); //                foretar 21 målinger av målingspinnen for resistansmåling for å få en mer nøyaktig måling. Dette over 210millisekunder totalt
                    delay(10); 
                  } 
                  ohmMaling = ohmMaling / 21;             //Deler på 21 for å få gjennomsnittet av målingen

                 utregningOhm=spenningKilde/4095;//                         Deler opp formelen for måling av resistans for å enklere kunne feilsøke
                 utregningOhm=utregningOhm*ohmMaling;//                         Deler opp formelen for måling av resistans for å enklere kunne feilsøke
                 utregningOhm=utregningOhm*resistans1;//                          Deler opp formelen for måling av resistans for å enklere kunne feilsøke
                 utregningOhm2=spenningKilde/4095;//                               Deler opp formelen for måling av resistans for å enklere kunne feilsøke
                 utregningOhm2=utregningOhm2*ohmMaling;//                              Deler opp formelen for måling av resistans for å enklere kunne feilsøke
                 utregningOhm2=spenningKilde-utregningOhm2;//                              Deler opp formelen for måling av resistans for å enklere kunne feilsøke
                 utregningOhm=utregningOhm/utregningOhm2;     //                               Deler opp formelen for måling av resistans for å enklere kunne feilsøke
                    



                  //____________________________________________kalibrering_____________________________________//  

                  if (ohmMaling>1150&&ohmMaling<1250)  //18ohm                 kalibreringen av resistanser er gjort ved bruk av verdiene på resistansene som står oppgitt
                  {kalibreringR=0.445;
                  Serial.println("VALGT -1");
                  }  
                  else if (ohmMaling>1250&&ohmMaling<1350)  //22ohm
                  {kalibreringR=0.50;
                  Serial.println("VALGT -3");
                  }  
                  else if (ohmMaling>1350&&ohmMaling<1450)  //27
                  {kalibreringR=0.55;
                  Serial.println("VALGT -4");
                  }  
                  else if (ohmMaling>1450&&ohmMaling<1550)  //33
                  {kalibreringR=0.60;
                  Serial.println("VALGT -5");
                  }  
                  else if (ohmMaling>1650&&ohmMaling<1750)  //47
                  {kalibreringR=0.69;
                  Serial.println("VALGT -5-1");
                  }  
                  else if (ohmMaling>1750&&ohmMaling<1850)  //Ukalibrert, grunnet mangel på resistor
                  {kalibreringR=0.72;
                  Serial.println("VALGT -6");
                  }  
                  else if (ohmMaling>1850&&ohmMaling<1950)  //68
                  {kalibreringR=0.765;
                  Serial.println("VALGT -6-1");
                  }  
                  else if (ohmMaling>1950&&ohmMaling<2050)  //Ukalibrert, grunnet mangel på resistor
                  {kalibreringR=0.80;
                  Serial.println("VALGT -7");
                  }  
                  else if (ohmMaling>2050&&ohmMaling<2150)  //82
                  {kalibreringR=0.81;
                  Serial.println("VALGT -8");
                  }  
                  else if (ohmMaling>2150&&ohmMaling<2400)  //Ukalibrert, grunnet mangel på resistor
                  {kalibreringR=0.87;
                  Serial.println("VALGT -9");
                  }  

                  else if (ohmMaling>1550&&ohmMaling<1650)  //39ohm
                  {kalibreringR=0.64;
                  Serial.println("VALGT -10");
                  }  
                    
                                                          //150ohm
                  else if (ohmMaling>2400&&ohmMaling<2600)  
                  {kalibreringR=0.91;
                  Serial.println("VALGT FØRSTE");
                  }                   

                  else if (ohmMaling>2600&&ohmMaling<2800)  //180
                  {kalibreringR=0.925;
                  Serial.println("VALGT ANDRE");
                  }  

                  else if (ohmMaling>2800&&ohmMaling<3000)  //218
                  {kalibreringR=0.95;
                  Serial.println("VALGT TREDJE");
                  }  
                  else if (ohmMaling>3000&&ohmMaling<3200)  //270
                  {kalibreringR=0.96;
                  Serial.println("VALGT FJERDE");
                  }  
                                                          //390
                  else if (ohmMaling>3200&&ohmMaling<3400)  
                  {kalibreringR=0.965;
                  Serial.println("VALGT FEMTE");
                  } 
                  else if (ohmMaling>3400&&ohmMaling<3500)  //470
                  {kalibreringR=0.97;
                  Serial.println("VALGT FEMTE");
                  } 
                  //_____________________________________________________________________________________________________________________//   

                  utregningOhm=utregningOhm*kalibreringR; //benytter kalibreringsfaktoren til å gange ut den utregnede motstandsverdien for å få en nøyaktig måling


                Serial.println("Ref - 1: ");
                Serial.println(String("utregnet ohm")+String(utregningOhm));
                      if (utregningOhm>999)//                                 Dersom verdien overstiger 1000 settes ohm til Kohm.
                      {
                        kilo=true;//                                    skifter kilo til "true" når resistansverdien skal skrives ut i kilo ohm
                        utregningOhm=utregningOhm/1000;
                          if (utregningOhm>999999)//                                 Dersom verdien overstiger 99999 er motstanden uendelig og motstand settes dermed til 0
                          {
                            utregningOhm=0;
                          }
                        Serial.println(String(utregningOhm)+String("KΩ"));//                                 Dersom motstanden er over 999 skal serial monitore printe ut "KΩ"
                      }
                        else
                        {
                          oled.clearDisplay();
                          Serial.println(String(utregningOhm)+String("Ω"));//                                 Dersom motstanden ikke er over 999 skal serial monitoren printe ut "Ω"
                        }
              }
            }
          if (valgtResistans==0)//                                 Samme kode som over hvor neste referansemotstand blir valgt
                 {
                        pinMode(res1,INPUT);
                        pinMode(res2,OUTPUT);
                        pinMode(res3,INPUT);
                        pinMode(res4,INPUT);
                        digitalWrite(res1,LOW);
                        digitalWrite(res2,HIGH);
                        digitalWrite(res3,LOW);
                        digitalWrite(res4,LOW);
                        ohmMaling=analogRead(35);
                        Serial.println(String("ref2 ")+String(ohmMaling));
                              if (ohmMaling>1700&&ohmMaling<3550)
                                {
                                      
                                      valgtResistans=2;

                   for (int i = 0; i < 20 ; i++)
                  { 
                    ohmMaling = ohmMaling + analogRead(35); 
                    delay(10); 
                  } 
                  ohmMaling = ohmMaling / 21; 

                                      //utregningOhm=(((spenningKilde/4096)*ohmMaling)*resistans2)/(spenningKilde-((spenningKilde/4096)*ohmMaling));
                                      utregningOhm=spenningKilde/4095;
                                        utregningOhm=utregningOhm*ohmMaling;
                                          utregningOhm=utregningOhm*resistans2;
                                            utregningOhm2=spenningKilde/4095;
                                              utregningOhm2=utregningOhm2*ohmMaling;
                                                utregningOhm2=spenningKilde-utregningOhm2;

                                      utregningOhm=utregningOhm/utregningOhm2;

//_______________________________________________________kalibrering______________________________________________________________//
                                                          
                  if (ohmMaling>1800&&ohmMaling<1950) //560
                  {kalibreringR=0.67;
                  Serial.println("VALGT FØRSTE");
                  }             
                  if (ohmMaling>1950&&ohmMaling<2050) //680
                  {kalibreringR=0.705;
                  Serial.println("VALGT 1-1");
                  }          
                                           
                  else if (ohmMaling>2050&&ohmMaling<2150)  //Ukalibrert
                  {kalibreringR=0.72;
                  Serial.println("VALGT ANDRE");
                  }
                  else if (ohmMaling>2150&&ohmMaling<2250)  //820
                  {kalibreringR=0.725;
                  Serial.println("VALGT TREDJE");
                  } 
                  else if (ohmMaling>2250&&ohmMaling<2350)  //1000ohm 
                  {kalibreringR=0.765;
                  Serial.println("VALGT 3-1");
                  } 
                  else if (ohmMaling>2350&&ohmMaling<2450)  //Ukalibrert
                  {kalibreringR=0.79;
                  Serial.println("VALGT FJERDE");
                  } 
                  else if (ohmMaling>2450&&ohmMaling<2550)  //1200
                  {kalibreringR=0.80;
                  Serial.println("VALGT FEMTE");
                  } 
                  else if (ohmMaling>2550&&ohmMaling<2650)  //Ukalibrert
                  {kalibreringR=0.815;
                  Serial.println("VALGT SJETTE");
                  } 
                  else if (ohmMaling>2650&&ohmMaling<2750)  //1500
                  {kalibreringR=0.83;
                  Serial.println("VALGT SYVENDE");
                  } 

                  else if (ohmMaling>2750&&ohmMaling<2850)  //1800 
                  {kalibreringR=0.840;
                  Serial.println("VALGT ÅTTENDE");
                  } 
                  else if (ohmMaling>2950&&ohmMaling<3050)  //2200
                  {kalibreringR=0.841;
                  Serial.println("VALGT 9");
                  }
                  else if (ohmMaling>3050&&ohmMaling<3150)  //2700
                  {kalibreringR=0.846;
                  Serial.println("VALGT 9-1");
                  }

                  else if (ohmMaling>3150&&ohmMaling<3250)  //3300
                  {kalibreringR=0.847;
                  Serial.println("VALGT FEMTE");
                  }
                  else if (ohmMaling>3250&&ohmMaling<3350)  //Ukalibrert 
                  {kalibreringR=0.847;
                  Serial.println("VALGT FEMTE");
                  }

                  else if (ohmMaling>3350&&ohmMaling<3450)  //3900
                  {kalibreringR=0.847;
                  Serial.println("VALGT 10");
                  } 
                  else if (ohmMaling>3450&&ohmMaling<3550)  //4700
                  {kalibreringR=0.847;
                  Serial.println("VALGT 10-1");
                  } 
//__________________________________________________________________________________________________________________________________________//   
                    utregningOhm=utregningOhm*kalibreringR;

                                      Serial.println("Ref - 2: ");
                                      Serial.println(String("utregnet ohm")+String(utregningOhm));
                                        if (utregningOhm>999)
                                        {
                                          kilo=true;
                                          utregningOhm=utregningOhm/1000;
                                            if (utregningOhm>999999)
                                            {
                                              utregningOhm=0;
                                            }
                                          Serial.println(String(utregningOhm)+String("KΩ"));
                                        }
                                      else
                                      {
                                        Serial.println(String(utregningOhm)+String("Ω"));
                                      }
                                  }
                  }
                if (valgtResistans==0)//                                 Samme kode som over hvor neste referansemotstand blir valgt
                {
                  pinMode(res1,INPUT);
                  pinMode(res2,INPUT);
                  pinMode(res3,OUTPUT);
                  pinMode(res4,INPUT);
                  digitalWrite(res1,LOW);
                  digitalWrite(res2,LOW);
                  digitalWrite(res3,HIGH);
                  digitalWrite(res4,LOW);
                  ohmMaling=analogRead(35);
                  Serial.println(String("ref3 ")+String(ohmMaling));

                      if (ohmMaling>1700&&ohmMaling<3500)
                      {

                          valgtResistans=3;

                    for (int i = 0; i < 20 ; i++)
                  { 
                    ohmMaling = ohmMaling + analogRead(35); 
                    delay(10); 
                  } 
                  ohmMaling = ohmMaling / 21; 
                          utregningOhm=(((spenningKilde/4095)*ohmMaling)*resistans3)/(spenningKilde-((spenningKilde/4095)*ohmMaling));

            //__________________________________________________________kalibrering________________________________________________________________//
                  if (ohmMaling>1700&&ohmMaling<1800) //5100 og 5600
                  {kalibreringR=0.73;
                  Serial.println("VALGT FØRSTE");
                  }          

                  if (ohmMaling>1800&&ohmMaling<1900) //ukalibrert
                  {kalibreringR=0.75;
                  Serial.println("VALGT 1-1");
                  }
                  if (ohmMaling>1900&&ohmMaling<2050) //6800
                  {kalibreringR=0.77;
                  Serial.println("VALGT 1-2");
                  }                                         
                                           
                  else if (ohmMaling>2050&&ohmMaling<2150)  // 8200
                  {kalibreringR=0.79;
                  Serial.println("VALGT ANDRE");
                  }
                  else if (ohmMaling>2150&&ohmMaling<2250)  //10k
                  {kalibreringR=0.83;
                  Serial.println("VALGT TREDJE");
                  } 
                  else if (ohmMaling>2250&&ohmMaling<2350)  //ukal
                  {kalibreringR=0.845;
                  Serial.println("VALGT 3-1");
                  } 
                  else if (ohmMaling>2350&&ohmMaling<2450)  //12k
                  {kalibreringR=0.86;
                  Serial.println("VALGT FJERDE");
                  } 
                  else if (ohmMaling>2450&&ohmMaling<2550)  //
                  {kalibreringR=0.87;
                  Serial.println("VALGT FEMTE");
                  } 
                  else if (ohmMaling>2550&&ohmMaling<2650)  //15k
                  {kalibreringR=0.88;
                  Serial.println("VALGT SJETTE");
                  } 
                  else if (ohmMaling>2650&&ohmMaling<2750)  //18k
                  {kalibreringR=0.90;
                  Serial.println("VALGT SYVENDE");
                  } 
                
                 else if (ohmMaling>2750&&ohmMaling<2850)  //ukalibrert
                 {kalibreringR=0.92;
                 Serial.println("VALGT ÅTTENDE");
                 } 
                 else if (ohmMaling>2850&&ohmMaling<2950)  //22k
                 {kalibreringR=0.935;
                 Serial.println("VALGT 8-1");
                 } 
                 else if (ohmMaling>2950&&ohmMaling<3050)  //27k
                 {kalibreringR=0.96;
                 Serial.println("VALGT 9");
                 }
                
                 else if (ohmMaling>3150&&ohmMaling<3250)  //33k
                 {kalibreringR=0.97;
                 Serial.println("VALGT 9-1");
                 }
                 else if (ohmMaling>3250&&ohmMaling<3350)  //39k
                 {kalibreringR=0.99;
                 Serial.println("VALGT 10");
                 } 
                 else if (ohmMaling>3350&&ohmMaling<3500)  //47k
                 {kalibreringR=1.04;
                 Serial.println("VALGT 10");
                 } 
                
          //// _____________________________________________________________________________________________________________________//   
                        utregningOhm=utregningOhm*kalibreringR;
                          Serial.println("Ref - 3: ");
                          Serial.println(String("utregnet ohm")+String(utregningOhm));
                              if (utregningOhm>999)
                              {
                                kilo=true;
                                utregningOhm=utregningOhm/1000;
                                if (utregningOhm>999999)
                                {
                                  utregningOhm=0;
                                }
                                Serial.println(String(utregningOhm)+String("KΩ"));
                              }
                              else
                              {
                                Serial.println(String(utregningOhm)+String("Ω"));
                              }
                        }
                }
                if (valgtResistans==0)//                                 Samme kode som over hvor neste referansemotstand blir valgt
                {
                    pinMode(res1,INPUT);
                    pinMode(res2,INPUT);
                    pinMode(res3,INPUT);
                    pinMode(res4,OUTPUT);
                    digitalWrite(res1,LOW);
                    digitalWrite(res2,LOW);
                    digitalWrite(res3,LOW);
                    digitalWrite(res4,HIGH);
                    ohmMaling=analogRead(35);

                        if (ohmMaling>1600&&ohmMaling<3850)
                        { 

                            valgtResistans=4;

                  for (int i = 0; i < 20 ; i++)
                  { 
                    ohmMaling = ohmMaling + analogRead(35); 
                    delay(10); 
                  } 
                  ohmMaling = ohmMaling / 21; 

                            utregningOhm=(((spenningKilde/4095)*ohmMaling)*resistans4)/(spenningKilde-((spenningKilde/4095)*ohmMaling));


                   //____________________________________________kalibrering_____________________________________//
                  if (ohmMaling>1600&&ohmMaling<1750) //56k
                  {kalibreringR=0.82;
                  Serial.println("VALGT FØRSTE");
                  }     
                  if (ohmMaling>1750&&ohmMaling<1850) //68k
                  {kalibreringR=0.885;
                  Serial.println("VALGT 1-1");
                  }
                  if (ohmMaling>1850&&ohmMaling<1950) //82k
                  {kalibreringR=0.92;
                  Serial.println("VALGT 1-2");
                  }    
                  if (ohmMaling>1950&&ohmMaling<2050) //100k
                  {kalibreringR=1.01;
                  Serial.println("VALGT 1-2-2");
                  }    
                  if (ohmMaling>2050&&ohmMaling<2150) //120k
                  {kalibreringR=1.05;
                  Serial.println("VALGT 1-3");
                  } 
                  if (ohmMaling>2150&&ohmMaling<2350) //150k
                  {kalibreringR=1.11;
                  Serial.println("VALGT 1-4");
                  }                   
                                                          
                  if (ohmMaling>2350&&ohmMaling<2650) //220k
                  {kalibreringR=1.20;
                  Serial.println("VALGT 1-5");
                  }                   
                                           
                  else if (ohmMaling>2650&&ohmMaling<2950)  // 270k
                  {kalibreringR=1.23;
                  Serial.println("VALGT ANDRE");
                  }
                  else if (ohmMaling>2950&&ohmMaling<3250)  // 330k og 390k
                  {kalibreringR=1.25;
                  Serial.println("VALGT TREDJE");
                  } 
                
                 //// _____________________________________________________________________________________________________________________//   
                 utregningOhm=utregningOhm*kalibreringR;
                            Serial.println("Ref - 4: ");
                            Serial.println(String("utregnet ohm")+String(utregningOhm));
                                  if (utregningOhm>999)
                                {
                                  kilo=true;
                                  utregningOhm=utregningOhm/1000;
                                  if (utregningOhm>999999)
                                  {
                                  utregningOhm=0;
                                  }
                                  Serial.println(String(utregningOhm)+String("KΩ"));
                                }
                                else
                                {
                                  Serial.println(String(utregningOhm)+String("Ω"));

                                }
                          }
                 }
                        if (valgtResistans==0)//                                Dersom ingen referansemotstander ble valgt vil motstanden være utenfor område for nøyaktig måling og motstanden settes til 0
                        {
                          utregningOhm=0;
                        }
Serial.println(String("ohm maling = ")+String(ohmMaling));
            batteriKapasitet();
            temperaturMaling();
          oledSkjerm();//                                         sender ut resultater til oled display
          rotasjon();//                                        Oppdatterer portmeter verdi for å skjekke om while loopen skal fortsette eller stanses
          delay(1000);
          
      }
}

//////////////////////////////////////////**DIODE**/////////////////////////////////////////////////////////////////////////
void diodeMaling1()
{
 while(portVerdi==4)
  {
       
       pinMode(res1,INPUT);
       pinMode(res2,OUTPUT);//                                        Ettersom diodemåling kan bruke samme målepinner som resistansmåling benyttes samme pinsene
       pinMode(res3,INPUT);//                                        Her settes referansemotstand 2 som en output som er på 1kohm og alle andre til input for å ikke påvirke resultatet.
       pinMode(res4,INPUT);
       digitalWrite(res1,LOW);
       digitalWrite(res2,HIGH);
       digitalWrite(res3,LOW);
       digitalWrite(res4,LOW);
       ohmMaling=analogRead(35); //                                        avleser pin 35 som er brukt som analog pin
        if (ohmMaling>4094)//                                        Dersom målingen er 4095 vil det ikke være noe spenningtap over dioden og utregnet diode blir satt til 0
        {
          utregnetDiode=0;
        }
        else//                                        Om ikke avlesningen er 4095 vil koden gå inn i formelen for å utregne spenningen over dioden
        {
      utregnetDiode=(ohmMaling*spenningKilde)/4095;//                                        Formel for utregning av spenning over dioden
      utregnetDiode=utregnetDiode+0.03; //                              kalibrering for indre resistans i kretsen som ikke er blitt med i beregningen
        }
//                             Spenninstap over diode før led dioden som skal måles
          
            Serial.println(String(utregnetDiode)+String("V"));
            batteriKapasitet();
            temperaturMaling();
            oledSkjerm();//                                        Printe ut resultater til oled skjerm
            rotasjon();
    delay(1000);
  }
}

//////////////////////////////////////////**Kontinuitet**/////////////////////////////////////////////////////////////////////////
          void kontinuitetsMaling()
          {
            while (portVerdi==8)
            {
              rotasjon();
              pinMode(res1,OUTPUT);//                                        Benytter samme målepinner som resistansmåling og setter referansemotstand 1 på 100ohm som høy og alle andre lav.
              pinMode(res2,INPUT);//                                        Benytter motstand på 100ohm for å kunne teste kontakt på lavest mulig motstand
              pinMode(res3,INPUT);
              pinMode(res4,INPUT);
              digitalWrite(res1,HIGH);//                                        Setter pin for referansemotstand 1 til høy og alle andre lav for å ikke påvirke resultatet
              digitalWrite(res2,LOW);
              digitalWrite(res3,LOW);
              digitalWrite(res4,LOW);
              
              ohmMaling=analogRead(35);
                  if (ohmMaling<4094)//                                        Dersom målingen er under 4095 som er max antallet for espen vil det 
                  {
                    oled.fillCircle(centerX, centerY, innerRadius, WHITE);//                                        Tegner opp en sirkel på oled skjermen for å simulere når det blir kontakt på målingen
                   // oled.display();//                                        Sender ut sirkelen til oled displayet
                    oled.setTextSize(1); 
                    oled.setTextColor(WHITE);
                    oled.setCursor(5, 5); 
                    oled.println(String(oledProsent)+String("%"));
                    oled.setTextSize(1); 
                    oled.setTextColor(WHITE);
                    oled.setCursor(100, 5); 
                    oled.println(String(oledTemp)+String("C"));
                          oled.setTextSize(1); 
                    oled.setCursor(40, 5); 
                    oled.println("Kontakt");
                    oled.display(); 
                    digitalWrite(buzzer,HIGH);//                                        Setter buzzerpinnen til høy for å pipe når det blir kontakt
                  }
                      else//                                        om målingen er på 4095 vil buzzer settes til low og sirkelen settes til ufyllt.
                      {            
                oled.clearDisplay();
                oled.drawCircle(centerX, centerY, outerRadius, WHITE);
                oled.setTextSize(1); 
                oled.setTextColor(WHITE);
                oled.setCursor(5, 5); 
                oled.println(String(oledProsent)+String("%"));
                oled.setTextSize(1); 
                oled.setTextColor(WHITE);
                oled.setCursor(100, 5); 
                oled.println(String(oledTemp)+String("C"));
                oled.setTextSize(1); 
                oled.setCursor(40, 5); 
                oled.println("Kontakt");
                oled.display();
                        digitalWrite(buzzer,LOW);                                 //slutter å pipe buzzeren når koden ikke måler kontakt
                      }
                      Serial.println(String("buzzing = ")+String(ohmMaling));//                                        Brukt under testing. Kan SLETTES
            }
                  batteriKapasitet();
                  temperaturMaling();
          }
//////////////////////////////////////////**Kapasitans**/////////////////////////////////////////////////////////////////////////
void kapasitansMaling()
{
  while (portVerdi==6)
  {
        mikro=true;                                   //resetter verdi for om det er mikro eller nano
        rotasjon();
        #define analogPin 35                          //avlesningspin
        #define chargePin 25                          //chargepin til 100k ohm resistans
        #define dischargePin 27                       //dischargepin til 10k ohm resistans
        #define resistorValue 10000.0F                      //definerer resistanspinnen som en verdi som kan kapasitansverdi kan divideres med
        unsigned long startTid;                                     //setter tidsvariabler til unsigned long for å begrense tiden den kan stå på minst mulig
        unsigned long bruktTid;                                     //setter tidsvariabler til unsigned long for å begrense tiden den kan stå på minst mulig
        float testTid;              //brukt til telling av mikrosekunder

        pinMode(chargePin,OUTPUT);                                      //setter en chargepin for hvilken pin som skal lade opp kondensatoren
        digitalWrite(chargePin,LOW);                                        //brukt til testing, ikke i bruk
        digitalWrite(chargePin,HIGH);                                               //setter chargepin til høy for å lade kondensatoren
        startTid = micros();                        //teller tiden i mikrosekunder for å få en nøyaktig måling av kapasitans
        //Serial.println(String("start= ")+String(startTid));
        Serial.println(analogRead(analogPin));
        while ((analogRead(analogPin) < 2588.04)&&portVerdi==6)                   //dersom analogpinnen avleser en verdi på over 2588,04 som er 62,3% av 4095 vil kondensatoren 
              {                                                                   //ha ladet seg opp 62,3% og koden kan gå videre til å gjøre utregninger og lade den ut
                Serial.println(analogRead(analogPin));              //avlesning av analogpinnen, brukt til testing
                rotasjon();                                   //teller rotasjon slik at ikke apperatet setter seg fast på kondensator måling
              } 
        testTid=micros();                                     //teller tiden i mikrosekunder for å få en nøyaktig måling av kapasitans
        //Serial.println(String("TEST = ")+String(testTid));
        bruktTid = testTid-startTid;                                    //teller sekundene det tok å lade opp kondensatoren til 63,2%
        //Serial.println(String("bruktTid= ")+String(bruktTid));                                                         
        C = (bruktTid/ resistorValue);                                        //deler tiden det tok å lade opp kondensatoren på den definerte resistansverdien brukt til utladning for å finne kapasitansen
        //Serial.println(String("KAPASITANS = ")+String(C*1000000));
        if (C>0.9)                                          //if løkke for når kapasitans blir målt i mikro
        {
          Serial.println(String((long)bruktTid) + String ("s    ") + String (C) + String("uF")); 
        }
        else if(C<0.9)                                                  //if løkke for når kapasitans blir målt i nano
        {
          mikro=false;
          Serial.println(String((long)bruktTid) + String ("s    ") + String (C*1000) + String("nF"));   //ganger med 1000 for å konvertere til nano
        }
        Serial.println(String("mikro = ")+String(mikro));                   //brukt til testing, kan SLETTES
        oledSkjerm();                                     //funksjon for å skrive ut verdiene til oledskjerm
        delay(500);


      digitalWrite(chargePin, LOW);                           //setter chargepin til low for å slutte å lade opp kondensator
      pinMode(dischargePin, OUTPUT);                              //setter dischargepun til output for å kunne lade ut kondensator
      digitalWrite(dischargePin, LOW);                                    //setter dischargepin til low for å ikke sende ut spenning på pinnen

        while((analogRead(analogPin) > 0)&portVerdi==6){                      //venter til kondensatoren er helt ladet ut med å gå videre i koden
          rotasjon();                                                             //setter inn rotasjonsfunksjon for å ikke sette seg fast i while loopen dersom kondensator ikke blir målt
      } 

      pinMode(dischargePin, INPUT);                                             //setter dischargepin til en input når koden skal resettes for å gi ut spenning kun på chargepinnen
            batteriKapasitet();             //setter på måling av spenning og temp
            temperaturMaling();             //setter på måling av spenning og temp
  }
}
//////////////////////////////////////////**Induktans**/////////////////////////////////////////////////////////////////////////
    void induktansMaling()
    {                                       
     while (portVerdi == 7) {
                              rotasjon();
                              pinMode(18, INPUT); 
                              ledcWrite(ledChannel, 200);
                              delay(5); 
                              ledcWrite(ledChannel, 0);
                              delayMicroseconds(100);
                              pulse = pulseIn(14, HIGH, 10000);
                              Serial.println(pulse);
                           
                              if (pulse > 0.1) { 
                           
                               kapasitans = 2.E-7; 
                               frekvens = 1.E6 / (2 * pulse);
                               induktanse = 1. / (kapasitans * frekvens * frekvens * 4. * 3.14159 * 3.14159);
                               induktanse *= 1E6;
                               induktanse_mH = induktanse / 1000;
                               Serial.print("Induktans: ");
                               Serial.print(induktanse_mH);
                               Serial.println(" mH");   
                                } 
    oledSkjerm();
    delay(1000);
     }
    }   

//////////////////////////////////////////**Termistor**//////////////////////////////////////////////////////////////////////
void temperaturMaling()
{
    pinMode(13,INPUT);
    temp=analogRead(13);//                                    Lese av porten hvor termistoren er koblet til og sette dette på variabelen "temp"
    resistansVerdi = resistansTemp * (4095 / temp - 1.0);//                 her gjøres regnestykke for spenningsdeleren hvor 10000 ohm ganges med 4095/temp-1. 4095 er avlesningen fra esp'en og temp er avlesningen til termistoren.
    logRes = log(resistansVerdi);//                                           "logr2" variabelen settes opp ved bruk av funksjonen log(), dette setter verdien inni parantesen til å bli logarytmisk
    temperatur = 1.0 / (therm1 + therm2*logRes + therm3*pow(logRes,3));//       temperatur blir ved dette regnestykke gjort om til grader oppgitt i kelvin, her bruker coefficient verdiene som er funnet fra kalkulator for termistorer som beskrevet øverst i koden
    temperatur=temperatur-273.15;//                                                 Her gjøres gradene i kelvin om til grader i celsius.
    oledTemp=temperatur;
}
//////////////////////////////////////////**BatteriProsent**/////////////////////////////////////////////////////////////////////////
void batteriKapasitet()
{
    float referanseSpenningbat=3.3;//                  Setter opp varialber for måling av batterikapasitet
    float malingSpenningbat;
    float referanse1=6800;//                Referansemotstandsverdiene benyttet til kretsen
    float referanse2=1000;//                Referansemotstandsverdiene benyttet til kretsen
    float vdrResbat;
    float spenningbat;
    float voltbat;

      malingSpenningbat=analogRead(4);    //må endres til esp pin

            vdrResbat = (referanse2 / (referanse2 + referanse1)); 
            for (int i = 0; i < 20 ; i++)                                   //må skjekkes, kan være denne skal være 21 om første gang den går gjennom ikke plusser på 1
            { 
              malingSpenningbat = malingSpenningbat + analogRead(4); 
              delay(3); 
            } 
            malingSpenningbat = malingSpenningbat / 21; 
            voltbat = ((malingSpenningbat * referanseSpenningbat) / 4095);   
              batteriProsent=(voltbat/1.15)*100;                                      //benytter 1.15 som spenningen espen leser av på 100%. Dette er en utregnet verdi fra spenningsdeleren i kretsen
            oledProsent=batteriProsent;//                                         Benytter oledProsent i oledskjerm funksjonen for å holde koden ryddig og lett leselig

}

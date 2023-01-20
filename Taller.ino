/////////////////// Definiciones previas ///////////////////
#include <Matrices.h>
#include <math.h>

#include <HX711.h>

#include <EasyBuzzer.h>

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// CLASES //
HX711 bascula;
LiquidCrystal_I2C lcd(0x27,16,2);

/////////////////// Puertos ///////////////////
#define Conmutador_Maestro 4  
#define Valvula_Manual 13
#define Sensor_20 14
#define Sensor_40 15
#define Sensor_60 16
#define Sensor_80 17
#define Sensor_100 A6
#define Boton 6

#define ELECTROVALVULA 5
#define INDICACION 11
#define ALERTA 12

#define BASCULA_DT 2
#define BASCULA_SCLK 3

/////////////////// Variables Globales ///////////////////
int state = 0;
int address = 2;
bool rebound = false;

double v1=0;
double v2=0;
double v3=0;

double A[3][3]={};
double P[3][1]={};

double Sensores[6]={0,4,8,12,16.3,20};
double Peso_Sensores[6]={0,0.96,2.12,3.39,4.75,6.26};

float peso_actual=0;
float peso_anterior=0;

float volumen_actual=0;
float volumen_anterior=0;

float volumen_lleno=0;
float volumen_vacio=0;

float volumen_ciclo=0;
float volumen_total=0;

int contador=0;

float tiempo_actual=0;
float tiempo_anterior=0;

float dwdt=0;
float dvdt=0;
float dwdt_inicial=0;
float dw_dt_acum = 0;

//Nivel estimado
double ecuacion_nivel = 0;

// VARIABLES DE BASCULA //
float factor_calibracion = -100670;

// CONFIGURACION PARALELISMO
bool ledState=LOW;
bool Buz=LOW;

unsigned long currentMillisBuzzer=0;
unsigned long previousMillisBuzzer=0;

unsigned long currentMillisLed=0;
unsigned long previousMillisLed=0;

unsigned long currentMillis_dw=0;
unsigned long previousMillis_dw=0;

unsigned long currentMillis_LCD=0;
unsigned long previousMillis_LCD=0;

unsigned int cnt = 0;
bool reset_volumen = 0;

unsigned long delay_a=0;
unsigned long delay_anterior=0;


//////////////////////////////////////////////////////////////


/////////////////// Funcion Especial - Regresion C ///////////////////
void RegresionCuadratica(double x[], double y[], double n){

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      rMultiplicacion[i][j]=0;
    }
  }

  double Xi=0;
  double Xi2=0;
  double Xi3=0;
  double Xi4=0;
  
  double Yi=0;
  double XiYi=0;
  double Xi2Yi=0;
  
  for(int i=0; i<n; i++)
    {
      Xi += x[i];
      Xi2 += pow(x[i], 2);
      Xi3 += pow(x[i], 3);
      Xi4 += pow(x[i], 4);

      Yi += y[i];
      XiYi += x[i]*y[i];
      Xi2Yi += pow(x[i], 2)*y[i];
    }

  A[0][0]=n;
  A[0][1]=Xi;
  A[0][2]=Xi2;

  A[1][0]=Xi;
  A[1][1]=Xi2;
  A[1][2]=Xi3;
  
  A[2][0]=Xi2;
  A[2][1]=Xi3;
  A[2][2]=Xi4;

  P[0][0]=Yi;
  P[1][0]=XiYi;
  P[2][0]=Xi2Yi;

  MatInversa(A);
  MatMultiplicacion(rInversa, P);
  
  v1 = rMultiplicacion[0][0];
  v2 = rMultiplicacion[1][0];
  v3 = rMultiplicacion[2][0];

  //  GUARDA EN LA MEMORIA
  Guardar_Memoria(v1);
  Guardar_Memoria(v2);
  Guardar_Memoria(v3);

  //Serial.print("La ecuación de la regresión cuadrática es la siguiente:\n y = %f X^2 + %f X + %f", v3, v2, v1);
  Serial.print("La ecuacion de la regresion cuadratica es la siguiente: "); 
  Serial.print(v3, 8); Serial.print(" , "); Serial.print(v2, 8); Serial.print(" , "); Serial.println(v1, 8);
}

/* MANEJO DE LA MEMORIA
   DIRECCIONES RESERVADAS
   0-1: direccion actual que ocupa al momento de guarda un valor
   2-5: coheficiente v1
   6-9: coheficiente v2
   10-13: coheficiente v3
 */
 
//  LEE DATO EN LA MEMORIA CON LA DIRECCION ESPECIFICADA
double Leer_Memoria(int address){
  double aux = 0;
  EEPROM.get(address, aux);
  return aux;
}

//  GUARDA UN DATO EN LA MEMORIA
void Guardar_Memoria(double &valor) {

  //  COMPRUEBA QUE LA DIRECCION NO SUPERE EL LIMITE ESTABLECIDO
  if(address < 1023){
    EEPROM.put(address, valor);
    address += sizeof(valor);
    EEPROM.put(0, address);
  }

  else{
    address = 26;
    EEPROM.put(address, valor);
    address += sizeof(valor);
    EEPROM.put(0, address);
  }
}

//  DETECTA CUANDO SE PULSA UN BOTON EVITANDO EL REBOTE
bool button(int &comprobar) {
  if(digitalRead(Boton) == HIGH) rebound = true;
  if(rebound == true && digitalRead(Boton) == LOW){
    rebound = false;
    comprobar = 0;
    return true;
  }
  else return false;
}

//  FUNCION AUXILIAR PARA MOSTRAR EN LA LCD
void LCD2(int a, int b, String ab, int c, int d, String cd){
    lcd.clear();
    lcd.setCursor(a,b);
    lcd.print(ab);
    lcd.setCursor(c,d);
    lcd.print(cd);
}

//  VERIFICA CUANDO LOS SENSORES SE ACTIVAN. EN CADA NIVEL CORRESPONDIENTE CAPTURA EL VALOR DEL PESO ACTUAL
void Peso_Sensor(bool &cr, int &comprobar){

  if(SA(Sensor_20) == true){

    if (SA(Sensor_40) == true){

      if (SA(Sensor_60) == true){

        if(SA(Sensor_80) == true){
          
          if (SA(Sensor_100) == true && cr == true){
            Peso_Sensores[5]= max(bascula.get_units(10),0);
            lcd.clear();
            LCD2(3,0,"NIVEL 100",3,1,"ALCANZADO");
            digitalWrite(ELECTROVALVULA, LOW);
            comprobar = 1;
          }

          else if(SA(Sensor_100) == false && cr == false){
            Peso_Sensores[4]= max(bascula.get_units(10),0);
            LCD2(4,0,"NIVEL 80",3,1,"ALCANZADO");
            cr = !cr;
          }
          else;
        }

        else if (SA(Sensor_80) == false && cr == true){
          Peso_Sensores[3]= max(bascula.get_units(10),0);
          LCD2(4,0,"NIVEL 60",3,1,"ALCANZADO");
          cr = !cr;    
        }
        else;
      }

      else if (SA(Sensor_60) == false && cr == false){
        Peso_Sensores[2]= max(bascula.get_units(10),0);
        LCD2(4,0,"NIVEL 40",3,1,"ALCANZADO");
        cr = !cr;
      }
      else;
    }

    else if (SA(Sensor_40) == false && cr == true){
      Peso_Sensores[1]= max(bascula.get_units(10),0);
      LCD2(4,0,"NIVEL 20",3,1,"ALCANZADO");
      cr = !cr;
    }
    else;
  }
  else;
}

//  REALIZA LA CALIBRACION INICIAL DEL SISTEMA (PROCEDIMIENTO PARA CALCULAR LOS COEFICIENTES DE LA REGRESION)
void Calibracion_Inicial(int &comprobar){

  //  VARIABLE DE CONTROL
  bool cr = true;

  //  REALIZA EL PROCESO
  if (comprobar != 1){
    
    lcd.clear();
    LCD2(1,0,"CONFIGURACION",4,1,"INICIAL");
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("LLENANDO TANQUE");
    Peso_Sensores[0]= max(bascula.get_units(10),0);

    while (comprobar != 1) {

      //  LLENAR EL TANQUE
      if (digitalRead(Valvula_Manual) == HIGH){
        Peso_Sensor(cr, comprobar);
        digitalWrite(ELECTROVALVULA, 0);
      }
      
      else if (digitalRead(Valvula_Manual) == LOW){
        LCD2(3,0,"POR FAVOR",3,1,"CERRAR VM");
        digitalWrite(ELECTROVALVULA, 1);
        delay(1000);
        lcd.clear();
      }
      else;
    }

    //  REGRESION CUADRATICA
    RegresionCuadratica(Peso_Sensores, Sensores,6);
    cr = false;
    LCD2(1,0,"CONFIGURACION",3,1,"FINALIZADA");
    delay(2000);
    LCD2(1,0,"ABRIR VALVULA",4,1,"MANUAL");
  }

  //  CARGA LOS COEFICIENTES GUARDADOS EN LA EEPROM
  else if(comprobar == 1, cr == true){
    v1 = Leer_Memoria(address);
    address += sizeof(v1);
    v2 = Leer_Memoria(address);
    address += sizeof(v2);
    v3 = Leer_Memoria(address);
    address += sizeof(v3);
  }
  else;
}

bool dW_dt(){

  float x=(0.15)*dwdt_inicial;
  if(dwdt>=x && dwdt<=0){
    return true;
  }else{
    return false;
  }

}

bool Nivel_Estimado(){
  if(((ecuacion_nivel*100)/20) >= 90){
    return true;
  }else{
    return false;
  }


  /*if(digitalRead()==1){
    return true;
  }else{
    return false;
  }*/

}



void mostrarElectroLCD()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Electrovalvula");
  lcd.setCursor(0,1);
  lcd.print("encendida");
  lcd.clear();
}

//  MUESTRA LAS VARIABLES DEL ENTORNO
void mostrarLCD()
{
  lcd.clear();
  //  MASA
    lcd.setCursor(0,0);
    lcd.print("m:");
    lcd.setCursor(2,0);
    lcd.print(peso_actual);

  //  VARIACION DE PESO
    lcd.setCursor(8,0);
    lcd.print("dW:");
    lcd.setCursor(11,0);
    lcd.print(dwdt);

  //  CAUDAL
    lcd.setCursor(0,1);
    lcd.print("Q:");
    lcd.setCursor(2,1);
    lcd.print(dvdt);

  //  NIVEL ESTIMADO
    lcd.setCursor(8,1);
    lcd.print("N:");
    lcd.setCursor(11,1);
    lcd.print(constrain( ((ecuacion_nivel*100)/20), 0,100),0); lcd.print("%");

  /*lcd.clear();
  unsigned long t0 = 0;
  int periodo = 2000;
  t0 = millis();
  while(millis() < t0+periodo)
  {
    // espere [periodo] milisegundos
  
    
  //masa
    lcd.setCursor(0,0);
    lcd.print("m:");
    lcd.setCursor(2,0);
    lcd.print(peso_actual);
  //variacion peso
    lcd.setCursor(8,0);
    lcd.print("dW:");
    lcd.setCursor(11,0);
    lcd.print(dwdt);
  //caudal
    lcd.setCursor(0,1);
    lcd.print("Q:");
    lcd.setCursor(2,1);
    lcd.print(dvdt);
  //Nivel estimado
    lcd.setCursor(8,1);
    lcd.print("N:");
    lcd.setCursor(11,1);
    lcd.print((ecuacion_nivel*100)/17);
  }
  
  periodo = 3000;
  lcd.clear();
  if(millis() < t0+periodo)
  {
      //Limpieza salto de pantalla
    
  
    lcd.setCursor(0,0);
    //Peso jugo
    lcd.print("Peso turno:");
    lcd.setCursor(12,0);
    lcd.print(volumen_total);

  }
  */

}

// MUESTRA EL PESO DEL TURNO
void mostrarLCD2()
{
  lcd.clear();
  lcd.setCursor(0,0);
  //  PESO JUGO
  lcd.print("Peso turno:");
  lcd.setCursor(12,0);
  lcd.print(volumen_total);
}

bool SA(int x){
  float voltaje=(analogRead(x)*(5.0 / 1023.0));
  
  if(voltaje<=3.10){
    return true;
  } else {
    return false;
  }
}


/*void Parpadeo(){
  Serial.print("PARPADEO");
  currentMillisLed = millis();

  digitalWrite(INDICACION, 1);

  if(currentMillisLed - previousMillisLed >= 1100){

    previousMillisLed=currentMillisLed;
    digitalWrite(INDICACION, 0);
  }
}*/

void ALARMA(){
  EasyBuzzer.beep(1500,1);
}

void setup() {
  Serial.begin(4800);

  //  CONFIGURACION DE LOS PINES
  pinMode(Conmutador_Maestro, INPUT);
  pinMode(Valvula_Manual, INPUT);
  pinMode(Sensor_20, INPUT);
  pinMode(Sensor_40, INPUT);
  pinMode(Sensor_60, INPUT);
  pinMode(Sensor_80, INPUT);
  pinMode(Sensor_100, INPUT);
  pinMode(Boton, INPUT);
  pinMode(ELECTROVALVULA, OUTPUT);
  pinMode(INDICACION, OUTPUT);
  pinMode(ALERTA, OUTPUT);


  //CONFIGURACION BUZZER
  EasyBuzzer.setPin(12);

  //CONFIGURACION BASCULA 
  bascula.begin(BASCULA_DT, BASCULA_SCLK);
  bascula.tare();
  long zero_factor = bascula.read_average();
  //
  bascula.set_scale(factor_calibracion); //Funcion para obtener el peso//

  // FUNCION PARA LA REGRESION CUADRATICA
  //RegresionCuadratica(Peso_Sensores, Sensores,6);

  //Configuración LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Bienvenido");
  delay(2000);
  lcd.clear();
  LCD2(0,0,"Desea realizar",0,1,"la calibracion?");
  delay(2000);
  int comprobar = 1;
  LCD2(0,0,"Pulsar Boton: SI",0,1,"No Pulsar: NO");
  delay(3500);
/*
  //Variables auxiliares
  unsigned long pmC = 0;
  bool aux = false;

  //Esperar al usuario
  while (aux == false){
    if (button(comprobar) == true) break;
    if(millis() - pmC >= 8000){
      pmC = millis();
      aux = true;
    }
  }*/

    if(digitalRead(Boton)==true){
    comprobar=0;
  } else {
    comprobar=1;
  }

  //Calibracion o cargar coeficientes
  Calibracion_Inicial(comprobar);

  //Configuracion de la direccion en memoria
  address = (int)Leer_Memoria(0);

  delay(2000);
  
}

void loop() {
  EasyBuzzer.update();
  ecuacion_nivel = (v1 + v2*peso_actual + v3*pow(peso_actual,2)); 

  peso_actual=max(bascula.get_units(),0); // TENER ENCUENTA A LA HORA DE CALIBRAR AL CELDA!!!!!!!
  volumen_actual=(((bascula.get_units())/0.998)); // Densidad del Agua  0,998 g/cm3

  tiempo_actual=(millis()/1000);

  delay_a = millis();

  if(delay_a - delay_anterior >= 1000){

    delay_anterior=delay_a;

    dwdt = (peso_actual-peso_anterior)/((tiempo_actual-tiempo_anterior));
    dvdt = (volumen_actual-volumen_anterior)/((tiempo_actual-tiempo_anterior));

    tiempo_anterior=tiempo_actual;
    peso_anterior=peso_actual;
    volumen_anterior=volumen_actual;
  }

  bool CM = digitalRead(Conmutador_Maestro);
  bool VM = !(digitalRead(Valvula_Manual));
  bool S20 = SA(Sensor_20);
  bool S40 = SA(Sensor_40);
  bool S60 = SA(Sensor_60);
  bool S80 = SA(Sensor_80);
  bool S100 = SA(Sensor_100);
  bool BT = digitalRead(Boton);
  
  if(CM==0 && VM==1 && S20==0 && S80==0){
    //NINGUNA SALIDA 
    state = 0;
  }

  if(state == 0 && CM==1 && VM==1 && S20==0 && S80==0){
    //NINGUNA SALIDA
    state = 1;
  }

  if(state == 1 && CM==1 && VM==0 && S20==0 && S80==0){
    //ACTIVA ELECTROVALVULA
    state = 2;
  }

  if(state == 2 && CM==1 && VM==0 && S20==1 && S80==0){
    //ACTIVA ELECTROVALVULA
    state = 3;
  }

  if(state == 3 && CM==1 && VM==0 && S20==1 && S80==1){
    //DESACTIVA ELECTRO VALVULA, INDICACION VISUAL
    //peso_anterior=peso;
    volumen_lleno=volumen_actual;
    state = 4;
  }


///////////////////////////////////////  ESTADO 4  80% ///////////////////////////////////////////////7
  if(state == 4 && CM==1 && VM==1 && S20==1 && S80==1){
    //APAGA INDICACION VISUAL HASTA QUE SE TERMINE DE vaciar EL TANQUE
    //variacion_peso_inicial = ((peso_actual-peso_anterior)/1);
    state = 7;
  }

  if(state==7 && cnt < 10){

    currentMillis_dw = millis();
    
    if(currentMillis_dw - previousMillis_dw >= 200){
      dw_dt_acum += dwdt;
      cnt++;
      previousMillis_dw=currentMillis_dw;

      Serial.println("TOMANDO DATOS DW/DT");
    }
  }

  if(state==7 && cnt==10){
    Serial.println("INICIAL DONE");
    dwdt_inicial=(dw_dt_acum/cnt);
  }

  //dwdt_inicial=(dw_dt_acum/cnt);

  if(state == 4 && CM==1 && VM==0 && S20==1 && S80==1 && Nivel_Estimado()==true){
    // EXISTE UN ERROR Y LA ELECTROVALVULA SIGUE PASANDO AGUA, SE DEBE DE GENERAR UNA ALERTA
    state = 6;
  }

  if(state==6){
    currentMillisBuzzer = millis();

    if(currentMillisBuzzer - previousMillisBuzzer >= 550){
      if (Buz == LOW) {
        ALARMA();
        Buz=HIGH;
      } else {
        EasyBuzzer.stopBeep();
        Buz = LOW;
      }
      Serial.println("ALARMA");
      previousMillisBuzzer = currentMillisBuzzer;
    }
  }


/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 5 ///////////////////////////////////////////////
  if(state == 5 && CM==1 && VM==1 && S20==0 && S80==0 && dW_dt()==true){
    //CIERRE LA VALVULA YA INDICACION INTERMITENTE
    state = 8;
  }

  if(state==8){

    currentMillisLed = millis();

    if(currentMillisLed - previousMillisLed >= 20){
      
      
      if (ledState == LOW) {
      ledState = HIGH;
      } else {
      ledState = LOW;
      }

      digitalWrite(INDICACION, ledState);
      Serial.println("PARPADEO");
      previousMillisLed = currentMillisLed;
    }
  }

  if(state == 1 && CM==0 && VM==1 && S20==0 && S80==0){
    state = 0;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 6 ///////////////////////////////////////////////
  if(state == 6 && CM==1 && VM==1 && S20==1 && S80==1 && Nivel_Estimado()==true){
    //indicacion visual
    volumen_lleno=volumen_actual;
    state = 7;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 7 ///////////////////////////////////////////////
  if(state == 7 && CM==1 && VM==1 && S20==0 && S80==0){
    //INDICACION VISUAL
    state = 5;
  }

  if(state == 7 && CM==0 && VM==1 && S20==0 && S80==0){
    state = 0;
  } 
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 8 ///////////////////////////////////////////////
if(state == 8 && CM==1 && VM==0 && S20==0 && S80==0){
    //ENCIENDA ELECTROVALVULA
    state = 2;
  }

if(state == 8 && CM==0 && VM==0 && S20==0 && S80==0){
    //APAGADO EL COMUTADOR MAESTRO, VAYA AL ESTADO 0
    state = 0;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////// SALIDAS //////////////////////////////////////
  switch(state){
  case 0:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    reset_volumen=0;
    cnt=0;
    dwdt_inicial=0;
    dw_dt_acum=0;
    break;

  case 1:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 1);
    digitalWrite(ALERTA, 0);
    break;

  case 2:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    //mostrarElectroLCD();
    reset_volumen=0;
    cnt=0;
    dwdt_inicial=0;
    dw_dt_acum=0;
    break;

  case 3:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    //mostrarElectroLCD();
    break;

  case 4:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 1);
    digitalWrite(ALERTA, 0);
    break;

  case 5:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 1);
    digitalWrite(ALERTA, 0);
    break;

  case 6:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 1);
    //ALARMA();
    break;

  case 7:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 0);
    digitalWrite(INDICACION, 0);
    EasyBuzzer.stopBeep();

  case 8:
    digitalWrite(ELECTROVALVULA,1);
    digitalWrite(ALERTA, 0);

    if(reset_volumen == 0){
      Serial.println("VOLUMEN CICLO FINAL");
      volumen_ciclo=(volumen_lleno-volumen_vacio);
      volumen_total+=volumen_ciclo;
      volumen_ciclo=0;
      reset_volumen = 1;
    }
    break;
 }
  Serial.println(" ");/*
 
  Serial.println("///////////////////////////////// ENTRADAS /////////////////////////////////");
  Serial.print("CM: "); Serial.print(CM); Serial.print("\t"); Serial.print("VM: "); Serial.print(VM); Serial.print("\t"); 
  Serial.print("S20: "); Serial.print(SA(Sensor_20)); Serial.print("\t");
  Serial.print("S40: "); Serial.print(SA(Sensor_40)); Serial.print("\t");
  Serial.print("S60: "); Serial.print(SA(Sensor_60)); Serial.print("\t");
  Serial.print("S80: "); Serial.print(SA(Sensor_80)); Serial.print("\t");
  Serial.print("S100: "); Serial.print(SA(Sensor_100)); Serial.print("\t   ");
  
  

  Serial.print("Nivel: "); Serial.print(Nivel_Estimado()); Serial.print("  "); Serial.print("1%: "); Serial.print(dW_dt()); Serial.println("\t");
  Serial.println("///////////////////////////////// VARIABLES /////////////////////////////////");
  Serial.print("Peso: "); Serial.print(peso_actual); Serial.print("kg"); Serial.print("\t\t"); Serial.print("Peso Anterior: "); Serial.print(peso_anterior); Serial.println("kg");
  Serial.print("Volumen: "); Serial.print(volumen_actual,5); Serial.print("m3"); Serial.print("\t"); Serial.print("Volumen Anterior: "); Serial.print(volumen_anterior,5); Serial.println("m3");
  Serial.print("Tiempo: "); Serial.print(tiempo_actual,5); Serial.print("\t"); Serial.print("Tiempo Anterior: "); Serial.println(tiempo_anterior,5);
  
  Serial.print("dw/dt: "); Serial.print(dwdt); Serial.print(" kg/s"); Serial.print("\t"); Serial.print("dv/dt: "); Serial.print(dvdt); Serial.println(" m3/s");

  Serial.println("//////////////////// ESTADOS ////////////////////");
  Serial.print("Estado Actual: "); Serial.println(state);

  Serial.print("INICIAL: "); Serial.println(dwdt_inicial);
  Serial.println(cnt);

  Serial.println(BT);*/

  //mostrar variables en pantalla
  currentMillis_LCD=millis();

  if(currentMillis_LCD - previousMillis_LCD >= 550){

    if(BT==1){
      mostrarLCD2();
    } else {
      mostrarLCD();
    }

    previousMillis_LCD = currentMillis_LCD;
  }

}

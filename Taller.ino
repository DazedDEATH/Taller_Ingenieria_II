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

#define ELECTROVALVULA 5
#define INDICACION 11
#define ALERTA 12

#define BASCULA_DT 2
#define BASCULA_SCLK 3



/////////////////// Variables Globales ///////////////////
int state = 0;
int address = 2;

double v1=0;
double v2=0;
double v3=0;

double A[3][3]={};
double P[3][1]={};

double Sensores[6]={0,3.4,6.8,10.2,13.6,17};
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

//Nivel estimado
double ecuacion_nivel = 0;

// VARIABLES DE BASCULA //
float factor_calibracion = -100670;

// CONFIGURACION PARALELISMO
bool ledState=LOW;
bool BuzzerState=LOW;

unsigned long currentMillisLed=0;
unsigned long previousMillisLed=0;
const long time = 200;

unsigned long currentMillisBuzzer=0;
unsigned long previousMillisBuzzer=0;
const long time2 = 1000;

//////////////////////////////////////////////////////////////



/////////////////// Funcion Especial - Regresion C ///////////////////
void RegresionCuadratica(double x[], double y[], double n){
  v1=0;
  v2=0;
  v3=0;

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

  //Guardar en la memoria
  Guardar_Memoria(v1);
  Guardar_Memoria(v2);
  Guardar_Memoria(v3);

  //Serial.print("La ecuación de la regresión cuadrática es la siguiente:\n y = %f X^2 + %f X + %f", v3, v2, v1);
  Serial.print("La ecuacion de la regresion cuadratica es la siguiente: "); 
  Serial.print(v3, 8); Serial.print(" , "); Serial.print(v2, 8); Serial.print(" , "); Serial.println(v1, 8);
}

/* MANEJO DE LA MEMORIA
   DIRECCIONES RESERVADAS
   0: verificacion si se ha realizado la primera calibracion (coheficientes)
   1: direccion actual que ocupa al momento de guarda un valor
   2-9: coheficiente v1
   10-7: coheficiente v2
   18-25: coheficiente v3
 */
 
double Leer_Memoria(int address){
  double aux = 0;
  EEPROM.get(address, aux);
  return aux;
}

void Guardar_Memoria(double &valor) {
  if(address < 125){
    EEPROM.update(address, valor);
    address += sizeof(valor);
    EEPROM.update(1, address);
  }
  else{
    address = 26;
    EEPROM.update(address, valor);
    address += sizeof(valor);
    EEPROM.update(1, address);
  }
}

void LCD2(int a, int b, String ab, int c, int d, String cd){
    lcd.clear();
    lcd.setCursor(a,b);
    lcd.print(ab);
    lcd.setCursor(c,d);
    lcd.print(cd);
}

void Peso_Sensor(bool &cr, int &comprobar){

  if(SA(Sensor_20) == true && SA(Sensor_40) == false){
    Peso_Sensores[1]= max(bascula.get_units(),0);
    if (cr == true) {LCD2(4,0,"NIVEL 20",3,1,"ALCANZADO"); cr = !cr;}
  }

  else if(SA(Sensor_40) == true && SA(Sensor_60) == false){
    Peso_Sensores[2]= max(bascula.get_units(),0);
    if (cr == false) {LCD2(4,0,"NIVEL 40",3,1,"ALCANZADO"); cr = !cr;}
  }

  else if(SA(Sensor_60) == true && SA(Sensor_80) == false){
    Peso_Sensores[3]= max(bascula.get_units(),0);
    if (cr == true) {LCD2(4,0,"NIVEL 60",3,1,"ALCANZADO"); cr = !cr;}
  }

  else if(SA(Sensor_80) == true && SA(Sensor_100) == false){
    Peso_Sensores[4]= max(bascula.get_units(),0);
    if (cr == false) {LCD2(4,0,"NIVEL 80",3,1,"ALCANZADO"); cr = !cr;}
  }

  else if(SA(Sensor_100) == true && SA(Sensor_80) == true){
    Peso_Sensores[5]= max(bascula.get_units(),0);
    lcd.clear();
    LCD2(3,0,"NIVEL 100",3,1,"ALCANZADO");
    digitalWrite(ELECTROVALVULA, LOW);
    cr = false;
    comprobar = 1;
  }
}

void Calibracion_Inicial(){
  int comprobar=0;
  EEPROM.get(0,comprobar);
  lcd.print(EEPROM.read(0));
  delay(2000);
  int aux = 1;
  bool cr = true;

  if (comprobar != aux){
    lcd.clear();
    LCD2(1,0,"CONFIGURACION",4,1,"INICIAL");
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("LLENANDO TANQUE");
    Peso_Sensores[0]= max(bascula.get_units(),0);
    digitalWrite(ELECTROVALVULA, HIGH);

    while (comprobar != 1) {
      if(digitalRead(Conmutador_Maestro) == HIGH && digitalRead(Valvula_Manual) == HIGH){
        Peso_Sensor(cr,comprobar);
      }
      else{
        lcd.clear();
        LCD2(3,0,"POR FAVOR",3,1,"CERRAR VM");
        delay(3000);
      }
    }

    RegresionCuadratica(Peso_Sensores, Sensores,6);
    EEPROM.write(0,comprobar);
    cr = false;
    LCD2(1,0,"CONFIGURACION",3,1,"FINALIZADA");
    EEPROM.get(0,aux);
    lcd.clear();
    lcd.print(comprobar);
  }

  else if(comprobar == 1 && cr == true){
    v1 = Leer_Memoria(address);
    address += sizeof(v1);
    v2 = Leer_Memoria(address);
    address += sizeof(v1);
    v3 = Leer_Memoria(address);
    address += sizeof(v1);
  }
}

bool dW_dt(){
  /*
  if(digitalRead(4)==1){
    return true;
  }else{
    return false;
  }*/


  float x=0.01*dwdt_inicial;
  if(dwdt<x){
    return true;
  }else{
    return false;
  }

}

bool Nivel_Estimado(){

  ecuacion_nivel = v1 + v2*peso_actual + v3*pow(peso_actual,2); 

  if(((ecuacion_nivel*100)/17) >= 90){
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


void mostrarLCD()
{
  lcd.clear();
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
    lcd.print(constrain( (ecuacion_nivel*100)/17, 0,100),0); lcd.print("%");


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

void mostarLCD2()
{
  lcd.clear();
    lcd.setCursor(0,0);
    //Peso jugo
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


void Parpadeo(){
  currentMillisLed = millis();

  digitalWrite(INDICACION, 1);
  if(currentMillisLed-previousMillisLed >= 1000){
    previousMillisLed=currentMillisLed;
    digitalWrite(INDICACION, 0);
  }
}

void ALARMA(){
  //currentMillisBuzzer = millis();

  EasyBuzzer.beep(1500,1);

  /*if(currentMillisBuzzer-previousMillisBuzzer >= 1000){
    previousMillisLed=currentMillisLed;
    EasyBuzzer.stopBeep();
  }*/
}



void setup() {
  Serial.begin(4800);
  //EEPROM.write(0,(int)10); //descomentar para volver a realizar prueba de calibracion
  pinMode(Conmutador_Maestro, INPUT);
  pinMode(Valvula_Manual, INPUT);
  pinMode(Sensor_20, INPUT);
  pinMode(Sensor_40, INPUT);
  pinMode(Sensor_60, INPUT);
  pinMode(Sensor_80, INPUT);
  pinMode(Sensor_100, INPUT);
  pinMode(ELECTROVALVULA, OUTPUT);
  pinMode(INDICACION, OUTPUT);
  pinMode(ALERTA, OUTPUT);


  //CONFIGURACION BUZZER
  EasyBuzzer.setPin(12);

  //CONFIGURACIÓN BASCULA 
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
  lcd.print("Bienvenido");

  //Calibracion inicial de la regresion cuadratica
  Calibracion_Inicial();

  //Configuracion de la direccion en memoria
  address = (int)Leer_Memoria(1);

  delay(2000);
  
}

void loop() {
  EasyBuzzer.update();

  peso_actual=max(bascula.get_units(),0); // TENER ENCUENTA A LA HORA DE CALIBRAR AL CELDA!!!!!!!
  volumen_actual=((peso_actual/9.8)/997); // Densidad del Agua en litros
  tiempo_actual=(millis()/1000);

  dwdt = (peso_actual-peso_anterior)/((tiempo_actual-tiempo_anterior));
  dvdt = ((volumen_actual-volumen_anterior) / ((tiempo_actual-tiempo_anterior)));

  bool CM = digitalRead(Conmutador_Maestro);
  bool VM = !(digitalRead(Valvula_Manual));
  bool S20 = SA(Sensor_20);
  bool S40 = SA(Sensor_40);
  bool S60 = SA(Sensor_60);
  bool S80 = SA(Sensor_80);
  bool S100 = SA(Sensor_100);
  
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
    state = 4;
    dwdt_inicial=dwdt;
  }


///////////////////////////////////////  ESTADO 4  80% ///////////////////////////////////////////////7
  if(state == 4 && CM==1 && VM==1 && S20==1 && S80==1){
    //APAGA INDICACION VISUAL HASTA QUE SE TERMINE DE LLENAR EL TANQUE
    //variacion_peso_inicial = ((peso_actual-peso_anterior)/1);
    state = 7;
  }

  if(state == 4 && CM==1 && VM==0 && S20==1 && S80==1 && Nivel_Estimado()==true){
    // EXISTE UN ERROR Y LA ELECTROVALVULA SIGUE PASANDO AGUA, SE DEBE DE GENERAR UNA ALERTA
    state = 6;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 5 ///////////////////////////////////////////////
  if(state == 5 && CM==1 && VM==1 && S20==0 && S80==0 && dW_dt()==true){
    //CIERRE LA VALVULA YA INDICACION INTERMITENTE
    state = 8;
  }

  if(state == 1 && CM==0 && VM==1 && S20==0 && S80==0){
    state = 0;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 6 ///////////////////////////////////////////////
  if(state == 6 && CM==1 && VM==1 && S20==1 && S80==1 && Nivel_Estimado()==true){
    //indicacion visual
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
    digitalWrite(INDICACION, 0);
    ALARMA();
    //SonidoAlerta(); 
    //digitalWrite(ALERTA, 1);
    break;

  case 7:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 0);
    EasyBuzzer.stopBeep();

    delay(100); //MEDIO SEGUNDO PARA ESPERAR EL AUMENTO DEL DW/DT Y QUE LA VARIABLE NO SEA 0
    
    while(contador<1){ //WHILE PARA TOMAR LOS DATOS SOLO 1 VEZ
      dwdt_inicial=dwdt;
      volumen_lleno=volumen_actual;
      Serial.print("dwdt_inicial: ");Serial.println(dwdt_inicial);
      contador++;
    }
    break;

  case 8:
    digitalWrite(ELECTROVALVULA,1);
    Parpadeo();
    digitalWrite(ALERTA, 0);

    volumen_ciclo=(volumen_lleno-volumen_vacio);
    volumen_total+=volumen_ciclo;
    break;
 }


  Serial.println(" ");
 
  peso_anterior=peso_actual;
  volumen_anterior=volumen_actual;
  tiempo_anterior=tiempo_actual;

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
  Serial.print("Tiempo: "); Serial.print(tiempo_actual,5); Serial.print("\t\t"); Serial.print("Tiempo Anterior: "); Serial.println(tiempo_anterior,5);
  
  Serial.print("dw/dt: "); Serial.print(dwdt); Serial.print(" kg/s"); Serial.print("\t"); Serial.print("dv/dt: "); Serial.print(dvdt); Serial.println(" m3/s");

  Serial.println("//////////////////// ESTADOS ////////////////////");
  Serial.print("Estado Actual: "); Serial.println(state);

  //mostrar vaiables en pantalla
  mostrarLCD();
}


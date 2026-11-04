
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <math.h>

// Los GPIO6 y GPIO8 del programa original no son adecuados en ESP32 clasico.
// En esa familia suelen estar reservados para la memoria SPI flash.
// Preferimos FALLAR EN LA COMPILACION antes que activar un pin peligroso.
#if defined(CONFIG_IDF_TARGET_ESP32)
#error                                                                         \
    "ESP32 clasico: revisar y cambiar GPIO 3/4/5/6/8. GPIO6/8 pueden pertenecer a la flash."
#endif

// ---------------------------------------------------------------------------
// 1. REMOTEXY Y BLUETOOTH: SIN CAMBIOS EN LA INTERFAZ
// ---------------------------------------------------------------------------
// GANANCIA: se usa la misma app, con su joystick y slider existentes.
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#include <RemoteXY.h>
#define REMOTEXY_BLUETOOTH_NAME "ROBOT_FUTBOL"

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = {255, 3,  0,   0,   0,   36,  0,  16,  202, 1,   5,
                           32,  17, 37,  30,  30,  2,   26, 24,  4,   128, 13,
                           23,  37, 6,   2,   78,  129, 0,  3,   3,   24,  6,
                           165, 66, 108, 117, 101, 32,  65, 110, 116, 0};

struct {
  int8_t joystick_1_x;  // -100..100: negativo=izq, positivo=der
  int8_t joystick_1_y;  // -100..100: negativo=atras, positivo=adelante
  int8_t slider_1;      // 0..100: LIMITE de velocidad, no segundo joystick
  uint8_t connect_flag; // RemoteXY informa si el celular esta conectado
} RemoteXY;
#pragma pack(pop)

// ---------------------------------------------------------------------------
// 2. PINES Y PWM
// ---------------------------------------------------------------------------
// SE MANTIENE el cableado del programa original, sujeto a verificacion.
constexpr uint8_t MOTOR_IZQ_1 = 3;
constexpr uint8_t MOTOR_IZQ_2 = 4;
constexpr uint8_t MOTOR_DER_1 = 5;
constexpr uint8_t MOTOR_DER_2 = 6;
constexpr uint8_t LED_ESTADO = 8;

// NO CAMBIA: el original YA TENIA PWM, a 20 kHz y con 8 bits.
// 8 bits => 0...255; 20 kHz => conmutacion fuera de la banda audible tipica.
// PWM es orden electrica, NO una medida real de RPM del motor.
constexpr uint32_t PWM_FRECUENCIA = 20000;
constexpr uint8_t PWM_BITS = 8;
constexpr int PWM_MAX = 255;

// La API de ESP32 core 2.x opera por CANALES.
// La API de core 3.x usa PINES con ledcAttach() / ledcWrite(pin, ...).
// MEJORA: codigo compatible con ambas familias de versiones.
constexpr uint8_t CANAL_IZQ_1 = 0;
constexpr uint8_t CANAL_IZQ_2 = 1;
constexpr uint8_t CANAL_DER_1 = 2;
constexpr uint8_t CANAL_DER_2 = 3;

// ---------------------------------------------------------------------------
// 3. PARAMETROS PARA AJUSTAR EN LAS PRUEBAS
// ---------------------------------------------------------------------------
constexpr int ZONA_MUERTA = 15; // 0..100; filtra movimientos pequenos

// 0.0 = curva lineal; 1.0 = curva cuadratica.
// 0.65 aporta mas precision cerca del centro, con maximo inalterado.
constexpr float SUAVIDAD_JOYSTICK = 0.65f;

// Si una rueda corre mas que la otra, bajar SOLO el factor de esa rueda.
// Valores iniciales de 1.0 NO corrigen diferencias hasta que los midan.
constexpr float FACTOR_IZQ = 1.00f;
constexpr float FACTOR_DER = 1.00f;

// Cambiar a true si el sentido fisico de la rueda no coincide con el mando.
constexpr bool INVERTIR_IZQ = false;
constexpr bool INVERTIR_DER = false;

// Dejar en 0 hasta medir el PWM minimo que vence la friccion del motor.
// Si se eleva, cuanto mas cerca del centro, menos fino puede ser el arranque.
constexpr int PWM_MINIMO = 0;

// Rampa por cada PERIODO_CONTROL_MS (no por vuelta del loop).
// Aumentar PASO_ACELERACION => acelera mas rapido.
// Aumentar PASO_DESACELERACION => reduce mas rapido.
constexpr int PASO_ACELERACION = 16;
constexpr int PASO_DESACELERACION = 36;
constexpr uint32_t PERIODO_CONTROL_MS = 10;
constexpr uint32_t PERIODO_LOG_MS = 200;

// Velocidad COMANDADA de cada rueda, con signo para el sentido.
// +255 adelante, -255 atras, 0 sin PWM.
int motorIzqActual = 0;
int motorDerActual = 0;
uint32_t ultimoControl = 0;
uint32_t ultimoLog = 0;

// Seguridad extra: una vez conectado el celular, exigimos poner primero
// el joystick en el centro antes de permitir mover los motores.
// GANANCIA: evita arrancar apenas reconecta con una orden anterior retenida.
bool esperandoCentro = true;

// ---------------------------------------------------------------------------
// 4. CAPA DE COMPATIBILIDAD DEL PWM ENTRE CORE 2.x Y 3.x
// ---------------------------------------------------------------------------
// ANTES: se llamaba directamente ledcSetup/ledcAttachPin/ledcWrite(canal).
// AHORA: ocultamos la diferencia de API en estas dos funciones.
bool iniciarPWM(uint8_t pin, uint8_t canal) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)canal; // los canales se asignan automaticamente
  return ledcAttach(pin, PWM_FRECUENCIA, PWM_BITS);
#else
  if (ledcSetup(canal, PWM_FRECUENCIA, PWM_BITS) <= 0)
    return false;
  ledcAttachPin(pin, canal);
  return true;
#endif
}

void escribirPWM(uint8_t pin, uint8_t canal, uint32_t duty) {
  duty = (uint32_t)constrain((int)duty, 0, PWM_MAX);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)canal;
  ledcWrite(pin, duty); // core 3.x: por PIN
#else
  (void)pin;
  ledcWrite(canal, duty); // core 2.x: por CANAL
#endif
}

// ---------------------------------------------------------------------------
// 5. CONTROL DE UN MOTOR CON PWM FIRMADO
// ---------------------------------------------------------------------------
// ANTES: setMotor(numero, estado, velocidad), con tres argumentos.
// AHORA: una sola orden -255..255 representa sentido Y potencia.
// Escribir primero AMBAS entradas en cero reduce la posibilidad de que
// queden accionadas a la vez al invertir el sentido.
// Nota: el tipo de frenado real depende del controlador de motores.
void escribirMotor(uint8_t pin1, uint8_t pin2, uint8_t canal1, uint8_t canal2,
                   int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  escribirPWM(pin1, canal1, 0);
  escribirPWM(pin2, canal2, 0);

  if (pwm > 0) {
    escribirPWM(pin1, canal1, (uint32_t)pwm);
  } else if (pwm < 0) {
    escribirPWM(pin2, canal2, (uint32_t)(-pwm));
  }
}

void aplicarMotores() {
  const int izq = INVERTIR_IZQ ? -motorIzqActual : motorIzqActual;
  const int der = INVERTIR_DER ? -motorDerActual : motorDerActual;

  escribirMotor(MOTOR_IZQ_1, MOTOR_IZQ_2, CANAL_IZQ_1, CANAL_IZQ_2, izq);
  escribirMotor(MOTOR_DER_1, MOTOR_DER_2, CANAL_DER_1, CANAL_DER_2, der);
}

// PARADA DE SEGURIDAD: pone las dos entradas de cada motor a duty 0.
// No equivale necesariamente a un frenado activo; puede haber inercia.
void paradaInmediata() {
  motorIzqActual = 0;
  motorDerActual = 0;
  aplicarMotores();
}

// ---------------------------------------------------------------------------
// 6. JOYSTICK PROPORCIONAL Y SENSIBILIDAD
// ---------------------------------------------------------------------------
// ANTES: si Y era mayor que X se iba adelante/atras; de lo contrario giraba.
// AHORA: X e Y se usan SIMULTANEAMENTE, sin perder las diagonales.
// MEJORA: distancia al centro -> cantidad de mando del motor.
float procesarEje(int valor) {
  valor = constrain(valor, -100, 100);
  const float magnitud = (float)abs(valor);

  if (magnitud <= ZONA_MUERTA)
    return 0.0f;

  // Reescalar de [zona muerta..100] a [0..1] para aprovechar todo el rango.
  const float normalizado = (magnitud - ZONA_MUERTA) / (100.0f - ZONA_MUERTA);

  // Mezcla lineal + cuadratica: menos brusca cerca del centro.
  const float suave = (1.0f - SUAVIDAD_JOYSTICK) * normalizado +
                      SUAVIDAD_JOYSTICK * normalizado * normalizado;
  return (valor > 0) ? suave : -suave;
}

// Transforma una orden [-1..1] en PWM firmado [-limite..+limite].
// El slider establece LIMITE, el joystick solicita una fraccion de ese limite.
int convertirAPWM(float orden, int limite) {
  orden = constrain(orden, -1.0f, 1.0f);
  limite = constrain(limite, 0, PWM_MAX);
  if (orden == 0.0f || limite == 0)
    return 0;

  const int signo = (orden > 0.0f) ? 1 : -1;
  const float magnitud = fabsf(orden);
  const int minimo = constrain(PWM_MINIMO, 0, limite);
  const int resultado = (int)lroundf(minimo + (limite - minimo) * magnitud);
  return signo * constrain(resultado, 0, limite);
}

// ---------------------------------------------------------------------------
// 7. RAMPA DE ACELERACION Y CAMBIO DE SENTIDO
// ---------------------------------------------------------------------------
// ANTES: al mover el joystick el PWM saltaba al valor del slider.
// AHORA: solo puede variar una cierta cantidad cada 10 ms.
// MEJORA: arranque y curvas menos bruscos; cambio de sentido en dos pasos:
//         primero reducir hasta 0, luego acelerar en sentido contrario.
// EXCEPCION: soltar el joystick, slider=0 o desconectar -> parada electrica
//            inmediata (sin rampa), priorizando la seguridad.
int actualizarConRampa(int actual, int objetivo) {
  if ((actual > 0 && objetivo < 0) || (actual < 0 && objetivo > 0)) {
    objetivo = 0; // detener antes de invertir el sentido
  }

  const int paso =
      (abs(objetivo) > abs(actual)) ? PASO_ACELERACION : PASO_DESACELERACION;

  if (actual < objetivo)
    return min(actual + paso, objetivo);
  if (actual > objetivo)
    return max(actual - paso, objetivo);
  return actual;
}

// ---------------------------------------------------------------------------
// 8. MEZCLA DIFERENCIAL: TODO CON UN SOLO JOYSTICK
// ---------------------------------------------------------------------------
// Ecuaciones:
//     ordenIzq = avance + giro
//     ordenDer = avance - giro
//
// Si Y=+1 y X=0 -> ambas ruedas adelante.
// Si Y=0 y X=+1 -> rueda izq adelante, rueda der atras (pivote a derecha).
// Si Y=+0.8 y X=+0.3 -> rueda izquierda mas rapida (curva derecha).
// Si Y=-1 y X=0 -> ambas ruedas atras.
// No existe un joystick independiente por motor: la mezcla es AUTOMATICA.
void controlarRobot() {
  const int x = constrain((int)RemoteXY.joystick_1_x, -100, 100);
  const int y = constrain((int)RemoteXY.joystick_1_y, -100, 100);
  const int slider = constrain((int)RemoteXY.slider_1, 0, 100);

  const float giro = procesarEje(x);
  const float avance = procesarEje(y);

  // MEJORA DE SEGURIDAD: desarmar el robot hasta que el mando quede neutro
  // tras cada inicio o reconexion BLE.
  if (esperandoCentro) {
    paradaInmediata();
    if (avance == 0.0f && giro == 0.0f) {
      esperandoCentro = false;
      Serial.println("Joystick centrado: control habilitado.");
    }
    return;
  }

  // Parada inmediata desde el punto de vista de las salidas de PWM.
  if ((avance == 0.0f && giro == 0.0f) || slider == 0) {
    paradaInmediata();
    return;
  }

  const uint32_t ahora = millis();
  if ((uint32_t)(ahora - ultimoControl) < PERIODO_CONTROL_MS)
    return;
  ultimoControl = ahora;

  // Mezclar avance y giro sin elegir una sola de cuatro direcciones.
  float ordenIzq = avance + giro;
  float ordenDer = avance - giro;

  // Normalizar evita exceder 1.0 cuando ambos ejes estan inclinados.
  const float maximo = max(1.0f, max(fabsf(ordenIzq), fabsf(ordenDer)));
  ordenIzq /= maximo;
  ordenDer /= maximo;

  const int limite = (int)lroundf(PWM_MAX * slider / 100.0f);

  // CALIBRACION: corregir diferencias entre las ruedas si aparecen.
  const int objetivoIzq = convertirAPWM(ordenIzq * FACTOR_IZQ, limite);
  const int objetivoDer = convertirAPWM(ordenDer * FACTOR_DER, limite);

  motorIzqActual = actualizarConRampa(motorIzqActual, objetivoIzq);
  motorDerActual = actualizarConRampa(motorDerActual, objetivoDer);
  aplicarMotores();

  // Log limitado a 5 mensajes/segundo; no saturar el monitor serie.
  if ((uint32_t)(ahora - ultimoLog) >= PERIODO_LOG_MS) {
    ultimoLog = ahora;
    Serial.print("X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" slider=");
    Serial.print(slider);
    Serial.print("% | PWMizq=");
    Serial.print(motorIzqActual);
    Serial.print(" PWMder=");
    Serial.println(motorDerActual);
  }
}

// ---------------------------------------------------------------------------
// 9. SETUP: INICIALIZACION DE MOTORES, LED Y BLE
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  bool ok = true;
  ok &= iniciarPWM(MOTOR_IZQ_1, CANAL_IZQ_1);
  ok &= iniciarPWM(MOTOR_IZQ_2, CANAL_IZQ_2);
  ok &= iniciarPWM(MOTOR_DER_1, CANAL_DER_1);
  ok &= iniciarPWM(MOTOR_DER_2, CANAL_DER_2);

  pinMode(LED_ESTADO, OUTPUT);
  digitalWrite(LED_ESTADO, LOW); // asume LED activo en ALTO

  paradaInmediata();

  if (!ok) {
    Serial.println(
        "ERROR: falla la configuracion del PWM. Verificar placa/pines.");
    while (true) {
      paradaInmediata();
      delay(100);
    }
  }

  RemoteXY_Init();
  Serial.println("ROBOT_FUTBOL: control diferencial iniciado.");
  Serial.println("Conectar BLE y CENTRAR joystick para habilitar motores.");
}

// ---------------------------------------------------------------------------
// 10. LOOP: BLUETOOTH + LED + PARADA EN DESCONEXION
// ---------------------------------------------------------------------------
void loop() {
  RemoteXY_Handler();

  const bool conectado = (RemoteXY.connect_flag == 1);
  digitalWrite(LED_ESTADO, conectado ? HIGH : LOW);

  if (!conectado) {
    // Se mantiene la proteccion original, ahora ademas exigimos recentrar
    // el joystick antes de volver a mover el robot tras reconectarse.
    esperandoCentro = true;
    paradaInmediata();
  } else {
    controlarRobot();
  }

  // Intervalo de servicio corto para no ralentizar innecesariamente el mando.
  // No garantiza latencia de BLE; depende de app, radio y libreria.
  RemoteXY_delay(2);
}

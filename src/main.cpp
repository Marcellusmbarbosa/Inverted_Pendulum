// ============================================================================
// Projeto Sarisa — Pêndulo Invertido v2.0
// Hardware: ESP32 WROOM DevKit 30 pinos + BTS7960 + 2 Encoders Incrementais
// Controle: State Feedback LQR + Swing-Up + Compensação de Atrito do Motor
// Ref: zjor/inverted-pendulum (adaptado para hardware Sarisa)
// ============================================================================
//
// Convenção de ângulo: theta = 0 → pêndulo para CIMA (equilíbrio instável)
//                      theta = ±PI → pêndulo para BAIXO
//
// Vetor de estados: [posição, ângulo, vel_carrinho, vel_angular]
//
// Máquina de estados:
//   IDLE → CALIBRATE → SWING_UP ↔ BALANCE → (FULL_STOP se fim de trilho)
//
// Comandos serial:
//   'S' = Start (calibra pêndulo + swing-up automático)
//   'L' = LQR direto (segure pêndulo em CIMA primeiro)
//   'C' = Calibrar zero do pêndulo por oscilação livre
//   'P' = Parar motor
//   'R' = Reset encoders + offset
//   Ajuste de parâmetros em tempo real (ver menu no boot)
// ============================================================================

#include <Arduino.h>
#include <ESP32Encoder.h>

// ========================== Pinagem do Hardware =============================

// Motor (Ponte H BTS7960) — RPWM(D15)=DIREITA, LPWM(D4)=ESQUERDA
#define PIN_RPWM 15
#define PIN_LPWM 4
#define PIN_REN  27
#define PIN_LEN  14

// Encoder do Pêndulo
#define ENC_PEND_A 32
#define ENC_PEND_B 33

// Encoder do Carrinho
#define ENC_CART_A 34
#define ENC_CART_B 35

// ======================== Objetos de Hardware ===============================

ESP32Encoder encCart;
ESP32Encoder encPend;

// ===================== Constantes de Calibração =============================
// Trilho: ~1.0m entre fins de curso, 11508 pulsos
// Pêndulo: 2050 pulsos/volta
const float K_POS = 0.0000869f;   // pulsos → metros
const float K_ANG = 0.003065f;    // pulsos → radianos
const float Ts    = 0.005f;       // Período de amostragem [s] (5 ms, 200 Hz)

// =================== Modelo do Motor DC ====================================
// Equação: dv/dt = -A*v + B*voltage + C*sign(v)
// Conversão: voltage = (control + A*v + C*sign(v)) / B
// AJUSTAR EMPIRICAMENTE! Valores iniciais conservadores:
float MOTOR_A = 10.0f;    // Atrito viscoso
float MOTOR_B = 1.5f;     // Ganho tensão → velocidade
float MOTOR_C = 1.0f;     // Atrito de Coulomb
const float V_SUPPLY = 12.0f;  // Tensão da fonte [V]

// ===================== Limites de Segurança =================================
const float POSICAO_LIMITE  = 0.45f;       // Limite do trilho [m]
const float THETA_THRESHOLD = PI / 10.0f;  // Ângulo para transição swing→balance (~18°)
const float MAX_PWM         = 250.0f;      // PWM máximo

// =================== Ganhos do Controlador ==================================
// LQR State Feedback Direto: u = Kx*x + Kv*v + Kth*theta + Kw*omega
// Convenção confirmada por diagnostico (26/03/2026):
//   Motor: u>0 → RPWM → carrinho vai DIREITA → x diminui
//   Pendulo: sentido horario (cai p/ direita) → theta diminui
//   Kth < 0 : pendulo cai direita (th<0) → u>0 → RPWM → carrinho pega
//   Kw  < 0 : vel angular CW (w<0) → u>0 → antecipa correção
//   Kx  > 0 : carrinho na direita (x<0) → u<0 → LPWM → volta ao centro
//   Kv  > 0 : carrinho movendo direita (v<0) → u<0 → freia
float Kth = -50.0f;   // Ganho do ângulo (NEGATIVO)
float Kw  = -8.0f;    // Ganho da velocidade angular (NEGATIVO)
float Kx  = 15.0f;    // Ganho da posição do carrinho (POSITIVO: centra)
float Kv  = 20.0f;    // Ganho da velocidade do carrinho (POSITIVO: amortece)

// Swing-up
float K_SWING = 13.0f;  // Ganho do swing-up por energia (calibrado 26/03)

// =================== Máquina de Estados ====================================
enum Estado {
    STATE_IDLE,
    STATE_CALIBRATE,
    STATE_SWING_UP,
    STATE_BALANCE,
    STATE_FULL_STOP
};

Estado estado_atual = STATE_IDLE;

// ====================== Variáveis de Estado ================================
float x = 0, last_x = 0, v = 0;           // Posição e velocidade do carrinho
float theta = 0, last_theta = 0, w = 0;   // Ângulo e velocidade angular
float last_w_filtered = 0;
float last_v_filtered = 0;                 // Filtro para vel. do carrinho
const float FILTER_ALPHA = 0.3f;           // Filtro passa-baixa (~14Hz a 200Hz)
float u_control = 0;                       // Saída do controlador
float u_voltage = 0;                       // Tensão efetiva aplicada

int cont_serial = 0;

// =================== Calibração do Zero (oscilação) ========================
bool  calibrando       = false;
bool  diagnostico      = false;  // Modo diagnostico (comando T)
float cal_offset_pend  = 0;
float cal_vel_anterior = 0;
float cal_ang_anterior = 0;     // Ângulo da amostra anterior (subamostrado)
float cal_soma_picos   = 0;
int   cal_n_picos      = 0;
int   cal_cont_serial  = 0;
int   cal_subsample    = 0;     // Contador para subamostrar a detecção de pico

// ======================== Funções Auxiliares ================================

// Normaliza ângulo para [-PI, +PI]
float normalizarAngulo(float ang) {
    while (ang > PI)  ang -= 2.0f * PI;
    while (ang < -PI) ang += 2.0f * PI;
    return ang;
}

// Saturação simétrica
float saturate(float val, float maxVal) {
    if (fabsf(val) > maxVal) return (val > 0) ? maxVal : -maxVal;
    return val;
}

// ====================== Funções do Motor ===================================

// Para o motor
void stopMotor() {
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
}

// Aciona motor com PWM direto (-255 a +255)
void driveMotor(float pwm_value) {
    int pwm = constrain((int)fabsf(pwm_value), 0, 255);
    if (pwm_value >= 0) {
        analogWrite(PIN_RPWM, pwm);
        analogWrite(PIN_LPWM, 0);
    } else {
        analogWrite(PIN_RPWM, 0);
        analogWrite(PIN_LPWM, pwm);
    }
}

// Converte controle → tensão → PWM, COMPENSANDO ATRITO DO MOTOR
// Baseado em zjor: voltage = (control + A*v + C*sign(v)) / B
void driveMotorWithControl(float control, float cart_velocity) {
    u_voltage = (control + MOTOR_A * cart_velocity + copysignf(MOTOR_C, cart_velocity)) / MOTOR_B;
    float pwm = MAX_PWM * u_voltage / V_SUPPLY;
    driveMotor(saturate(pwm, MAX_PWM));
}

// ====================== Funções de Leitura =================================

float getAngle() {
    float raw = encPend.getCount() * K_ANG;
    return normalizarAngulo(raw - cal_offset_pend);
}

float getCartPosition() {
    return encCart.getCount() * K_POS;
}

// ====================== Funções de Controle ================================

// Verifica se o pêndulo está perto da vertical p/ iniciar balance
bool isControllable(float th, float omega, float pos) {
    return (fabsf(th) < THETA_THRESHOLD) &&
           (fabsf(omega) < 3.0f) &&
           (fabsf(pos) < 0.30f);
}

// LQR State Feedback Direto — SEM OBSERVADOR
// u = Kx*x + Kv*v + Kth*theta + Kw*omega
float getBalancingControl(float pos, float vel, float th, float omega) {
    return Kx * pos + Kv * vel + Kth * th + Kw * omega;
}

// Swing-up por energia (baseado em zjor)
// Aplica impulso na direção oposta a omega*cos(theta) para injetar energia
float getSwingUpControl(float pos, float vel, float th, float omega) {
    float c = -copysignf(K_SWING, -omega * cosf(th));

    // Centralização suave do carrinho durante swing-up
    // Kx>0: pos negativo(direita) → c negativo → LPWM → volta ao centro
    c += 2.0f * pos + 1.0f * vel;

    // Proteção: se carrinho perto do limite do trilho, freia
    float lim = POSICAO_LIMITE * 0.7f;
    if ((pos <= -lim && c > 0) || (pos >= lim && c < 0)) {
        // Frenagem: força contrária à velocidade
        c = -(MOTOR_A * vel + copysignf(MOTOR_C, vel));
    }

    return c;
}

// ====================== Reset ==============================================

void resetarEstado() {
    encCart.clearCount();
    encPend.clearCount();
    x = last_x = v = last_v_filtered = 0;
    theta = last_theta = w = last_w_filtered = 0;
    u_control = u_voltage = 0;
    cont_serial = 0;
    stopMotor();
}

// ====================== Calibração (espera em repouso) =====================
// Espera o pêndulo parar (pendurado para baixo = PI)
// Depois calcula offset: valor_raw - PI
void calibrateWaitAtRest() {
    Serial.println("   Esperando pendulo parar (2s estavel)...");
    long lastReading;
    int estavel = 0;
    do {
        lastReading = encPend.getCount();
        delay(500);
        if (lastReading == encPend.getCount()) {
            estavel++;
            Serial.printf("   Estavel: %d/4\n", estavel);
        } else {
            estavel = 0;
        }
    } while (estavel < 4);

    float raw = encPend.getCount() * K_ANG;
    cal_offset_pend = raw - PI;
    Serial.printf(">> Pendulo em repouso. Raw=%.4f rad\n", raw);
    Serial.printf(">> Offset = %.4f rad (%.2f graus)\n",
                  cal_offset_pend, cal_offset_pend * 180.0f / PI);
}

// Imprime parâmetros atuais
void printParams() {
    Serial.println("\n--- PARAMETROS ATUAIS ---");
    Serial.printf("Motor: A=%.1f B=%.2f C=%.2f V=%.1f\n", MOTOR_A, MOTOR_B, MOTOR_C, V_SUPPLY);
    Serial.printf("LQR:   Kth=%.1f Kw=%.1f Kx=%.1f Kv=%.1f\n", Kth, Kw, Kx, Kv);
    Serial.printf("Swing: K=%.1f  Threshold=%.2f rad\n", K_SWING, THETA_THRESHOLD);
    Serial.printf("Offset: %.4f rad  Ts=%.3fs\n", cal_offset_pend, Ts);
    Serial.printf("Estado: %d\n", estado_atual);
    Serial.println("-------------------------\n");
}

// ============================== Setup ======================================

void setup() {
    Serial.begin(115200);

    // Motor parado
    pinMode(PIN_RPWM, OUTPUT);
    pinMode(PIN_LPWM, OUTPUT);
    stopMotor();

    // Habilita ponte H
    pinMode(PIN_REN, OUTPUT);
    pinMode(PIN_LEN, OUTPUT);
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, HIGH);

    // Encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encCart.attachFullQuad(ENC_CART_A, ENC_CART_B);
    encPend.attachFullQuad(ENC_PEND_A, ENC_PEND_B);
    encCart.clearCount();
    encPend.clearCount();

    Serial.println("============================================");
    Serial.println("  Sarisa — Pendulo Invertido v2.0");
    Serial.println("  State Feedback + Swing-Up + Compensacao");
    Serial.println("============================================");
    Serial.printf("  K_POS=%.7f  K_ANG=%.6f  Ts=%.3f\n", K_POS, K_ANG, Ts);
    Serial.printf("  Motor: A=%.1f B=%.1f C=%.1f\n", MOTOR_A, MOTOR_B, MOTOR_C);
    Serial.printf("  LQR: Kth=%.1f Kw=%.1f Kx=%.1f Kv=%.1f\n", Kth, Kw, Kx, Kv);
    Serial.println("============================================");
    Serial.println("Comandos:");
    Serial.println("  'S' = Start (calibra + swing-up automatico)");
    Serial.println("  'L' = LQR direto (segure pendulo em CIMA)");
    Serial.println("  'C' = Calibrar zero (oscilacao livre)");
    Serial.println("  'P' = Parar motor");
    Serial.println("  'R' = Reset encoders + offset");
    Serial.println("  'I' = Mostrar parametros atuais");
    Serial.println("  'T' = Diagnostico (mostra sensores ao vivo)");
    Serial.println("  'M' = Pulso motor RPWM  'N' = Pulso motor LPWM");
    Serial.println("  'A/a' = MOTOR_A +/-1   'B/b' = MOTOR_B +/-0.1");
    Serial.println("  'D/d' = MOTOR_C +/-0.1 '1/2' = Kth +/-5");
    Serial.println("  '3/4' = Kw +/-5   '5/6' = Kx +/-5");
    Serial.println("  '7/8' = Kv +/-5   '9/0' = K_SWING +/-1\n");

    estado_atual = STATE_IDLE;
}

// =============================== Loop ======================================

void loop() {

    // =============== Modo Diagnostico (comando T) ==========================
    if (diagnostico) {
        float pend_raw = encPend.getCount() * K_ANG;
        float cart_raw = encCart.getCount() * K_POS;
        long pend_pulsos = encPend.getCount();
        long cart_pulsos = encCart.getCount();

        static int diag_cnt = 0;
        if (++diag_cnt >= 20) {  // a cada 200ms
            diag_cnt = 0;
            Serial.printf("DIAG | Pend: %6ld pulsos = %7.3f rad (%6.1f graus) | Cart: %6ld pulsos = %6.3f m\n",
                          pend_pulsos, pend_raw, pend_raw * 180.0f / PI,
                          cart_pulsos, cart_raw);
        }

        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 'P' || cmd == 'p' || cmd == 'T' || cmd == 't') {
                diagnostico = false;
                Serial.println(">> Diagnostico encerrado.\n");
            }
        }

        delay(10);
        return;
    }

    // =============== Modo Calibração por Oscilação =========================
    if (calibrando) {
        unsigned long tempo_cal = micros();

        float x2_raw = encPend.getCount() * K_ANG;
        float x2_rad = normalizarAngulo(x2_raw);

        // Subamostrar a detecção de pico: calcular velocidade a cada 50ms (10 × 5ms)
        // Isso dá uma diferença de ângulo grande o suficiente para detectar mudança de sinal
        if (++cal_subsample >= 10) {
            cal_subsample = 0;
            float dt_sub = Ts * 10.0f;  // 50ms
            float vel_ang = normalizarAngulo(x2_rad - cal_ang_anterior) / dt_sub;
            cal_ang_anterior = x2_rad;

            // Detecta inversão de velocidade (pico/vale), ignora velocidade muito baixa (ruído)
            const float VEL_MIN = 0.05f;  // threshold mínimo [rad/s]
            const float PEAK_MAX = PI / 3.0f; // Só aceita picos perto do equilíbrio (~60°)
            if (fabsf(vel_ang) > VEL_MIN && fabsf(cal_vel_anterior) > VEL_MIN) {
                if (vel_ang * cal_vel_anterior < 0) {
                    if (fabsf(x2_rad) < PEAK_MAX) {
                        cal_soma_picos += x2_rad;
                        cal_n_picos++;
                        Serial.printf("  >> Pico #%d ACEITO: Ang=%.4f rad\n", cal_n_picos, x2_rad);
                    } else {
                        Serial.printf("  >> Pico IGNORADO (longe do equilibrio): Ang=%.4f rad\n", x2_rad);
                    }
                }
            }
            if (fabsf(vel_ang) > VEL_MIN) {
                cal_vel_anterior = vel_ang;
            }
        }

        // Log a cada 200ms (40 × 5ms)
        if (++cal_cont_serial >= 40) {
            cal_cont_serial = 0;
            float media = (cal_n_picos > 0) ? cal_soma_picos / cal_n_picos : 0;
            Serial.printf("CAL  | Ang:%.4f Vel:%.2f Picos:%d Media:%.4f\n",
                          x2_rad, cal_vel_anterior, cal_n_picos, media);
        }

        // Comandos durante calibração
        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 'P' || cmd == 'p' || cmd == 'F' || cmd == 'f') {
                if (cal_n_picos >= 4) {
                    cal_offset_pend = cal_soma_picos / cal_n_picos;
                    Serial.printf("\n>> CALIBRACAO OK! %d picos. Offset=%.4f rad (%.2f graus)\n",
                                  cal_n_picos, cal_offset_pend, cal_offset_pend * 180.0f / PI);
                } else {
                    Serial.println("\n>> Poucos picos (<4). Tente com mais oscilacao.");
                    cal_offset_pend = 0;
                }
                calibrando = false;
                estado_atual = STATE_IDLE;
                Serial.println("Envie 'S' para iniciar ou 'C' para recalibrar.\n");
            }
        }

        while (micros() - tempo_cal < (unsigned long)(Ts * 1000000));
        return;
    }

    // =============== Modo IDLE / FULL_STOP — comandos ======================
    if (estado_atual == STATE_IDLE || estado_atual == STATE_FULL_STOP) {
        if (Serial.available()) {
            char cmd = Serial.read();
            switch (cmd) {
                case 'S': case 's': {
                    Serial.println("\n>> MODO AUTOMATICO: Calibrando...");
                    resetarEstado();
                    calibrateWaitAtRest();
                    // Zeramos o carrinho depois da calibração
                    encCart.clearCount();
                    x = last_x = v = 0;
                    theta = last_theta = w = last_w_filtered = 0;
                    estado_atual = STATE_SWING_UP;
                    Serial.println(">> SWING-UP ATIVO! Pendulo sobe sozinho.\n");
                    break;
                }
                case 'L': case 'l': {
                    Serial.println("Zerando encoders... Segure pendulo em CIMA!");
                    delay(2000);
                    resetarEstado();
                    cal_offset_pend = 0;  // Zera offset — encoder ja esta no zero
                    estado_atual = STATE_BALANCE;
                    Serial.println(">> LQR BALANCE ATIVO! Solte o pendulo.\n");
                    break;
                }
                case 'C': case 'c': {
                    Serial.println("\n>> CALIBRACAO POR OSCILACAO");
                    Serial.println("   Solte o pendulo e deixe oscilar.");
                    Serial.println("   Envie 'P' ou 'F' para finalizar.\n");
                    encPend.clearCount();
                    last_theta = 0;
                    cal_vel_anterior = 0;
                    cal_ang_anterior = 0;
                    cal_soma_picos = 0;
                    cal_n_picos = 0;
                    cal_cont_serial = 0;
                    cal_subsample = 0;
                    calibrando = true;
                    break;
                }
                case 'R': case 'r': {
                    resetarEstado();
                    cal_offset_pend = 0;
                    estado_atual = STATE_IDLE;
                    Serial.println(">> Reset completo. Offset removido.\n");
                    break;
                }
                case 'I': case 'i': printParams(); break;
                case 'T': case 't': {
                    Serial.println("\n>> DIAGNOSTICO: Mova pendulo e carrinho com a mao.");
                    Serial.println("   Observe os sinais. Envie 'P' para sair.\n");
                    encPend.clearCount();
                    encCart.clearCount();
                    cal_offset_pend = 0;
                    diagnostico = true;
                    break;
                }
                case 'M': case 'm': {
                    Serial.println(">> Pulso RPWM (PWM=120) por 300ms...");
                    analogWrite(PIN_RPWM, 120);
                    analogWrite(PIN_LPWM, 0);
                    delay(300);
                    stopMotor();
                    Serial.printf("   Cart pos = %.3f m (%ld pulsos)\n",
                                  encCart.getCount() * K_POS, encCart.getCount());
                    break;
                }
                case 'N': case 'n': {
                    Serial.println(">> Pulso LPWM (PWM=120) por 300ms...");
                    analogWrite(PIN_RPWM, 0);
                    analogWrite(PIN_LPWM, 120);
                    delay(300);
                    stopMotor();
                    Serial.printf("   Cart pos = %.3f m (%ld pulsos)\n",
                                  encCart.getCount() * K_POS, encCart.getCount());
                    break;
                }
                // Ajuste de parâmetros do motor
                case 'A': MOTOR_A += 1.0f; Serial.printf("MOTOR_A = %.1f\n", MOTOR_A); break;
                case 'a': MOTOR_A = max(0.0f, MOTOR_A - 1.0f); Serial.printf("MOTOR_A = %.1f\n", MOTOR_A); break;
                case 'B': MOTOR_B += 0.1f; Serial.printf("MOTOR_B = %.2f\n", MOTOR_B); break;
                case 'b': MOTOR_B = max(0.1f, MOTOR_B - 0.1f); Serial.printf("MOTOR_B = %.2f\n", MOTOR_B); break;
                case 'D': MOTOR_C += 0.1f; Serial.printf("MOTOR_C = %.2f\n", MOTOR_C); break;
                case 'd': MOTOR_C = max(0.0f, MOTOR_C - 0.1f); Serial.printf("MOTOR_C = %.2f\n", MOTOR_C); break;
                // Ajuste de ganhos LQR
                case '1': Kth += 5.0f; Serial.printf("Kth = %.1f\n", Kth); break;
                case '2': Kth = max(0.0f, Kth - 5.0f); Serial.printf("Kth = %.1f\n", Kth); break;
                case '3': Kw += 5.0f; Serial.printf("Kw = %.1f\n", Kw); break;
                case '4': Kw = max(0.0f, Kw - 5.0f); Serial.printf("Kw = %.1f\n", Kw); break;
                case '5': Kx += 5.0f; Serial.printf("Kx = %.1f\n", Kx); break;
                case '6': Kx -= 5.0f; Serial.printf("Kx = %.1f\n", Kx); break;
                case '7': Kv += 5.0f; Serial.printf("Kv = %.1f\n", Kv); break;
                case '8': Kv -= 5.0f; Serial.printf("Kv = %.1f\n", Kv); break;
                case '9': K_SWING += 1.0f; Serial.printf("K_SWING = %.1f\n", K_SWING); break;
                case '0': K_SWING = max(1.0f, K_SWING - 1.0f); Serial.printf("K_SWING = %.1f\n", K_SWING); break;
                default: break;
            }
        }
        delay(10);
        return;
    }

    // =============== Loop de Controle Ativo (5ms / 200Hz) ==================
    unsigned long tempo_inicio = micros();

    // 1. Leitura dos sensores
    x     = getCartPosition();
    theta = getAngle();

    // 2. Velocidades por diferença finita + filtro passa-baixa
    float v_raw = (x - last_x) / Ts;
    v = FILTER_ALPHA * v_raw + (1.0f - FILTER_ALPHA) * last_v_filtered;
    float w_raw = normalizarAngulo(theta - last_theta) / Ts;
    w = FILTER_ALPHA * w_raw + (1.0f - FILTER_ALPHA) * last_w_filtered;

    last_x = x;
    last_theta = theta;
    last_v_filtered = v;
    last_w_filtered = w;

    // 3. Proteção de posição — carrinho no fim do trilho
    if (fabsf(x) >= POSICAO_LIMITE) {
        stopMotor();
        estado_atual = STATE_FULL_STOP;
        Serial.printf("SEGURANCA: Pos=%.3fm >= lim. Motor parado.\n", x);
        Serial.println("Envie 'S' para reiniciar ou 'R' para reset.\n");
        return;
    }

    // 4. Comando de parada durante operação
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'P' || cmd == 'p') {
            stopMotor();
            estado_atual = STATE_IDLE;
            Serial.println(">> PARADO. Envie 'S' para reiniciar.\n");
            return;
        }
    }

    // 5. Máquina de estados do controle
    switch (estado_atual) {

        case STATE_SWING_UP: {
            // Verifica se pode transicionar para balance
            if (isControllable(theta, w, x)) {
                estado_atual = STATE_BALANCE;
                cont_serial = 0;
                Serial.println("\n>> TRANSICAO: Swing-Up -> BALANCE!\n");
            } else {
                u_control = getSwingUpControl(x, v, theta, w);
                driveMotorWithControl(u_control, v);
            }

            // Log a cada 200ms (40 × 5ms)
            if (++cont_serial >= 40) {
                cont_serial = 0;
                Serial.printf("SWING | Ang:%.3f w:%.2f Pos:%.3f v:%.2f u:%.1f\n",
                              theta, w, x, v, u_control);
            }
            break;
        }

        case STATE_BALANCE: {
            // Se pêndulo saiu da zona controlável, volta para swing-up
            if (fabsf(theta) > THETA_THRESHOLD * 2.0f) {
                estado_atual = STATE_SWING_UP;
                cont_serial = 0;
                Serial.printf(">> PERDEU (Ang=%.3f). Voltando ao Swing-Up.\n", theta);
                break;
            }

            u_control = getBalancingControl(x, v, theta, w);
            driveMotorWithControl(u_control, v);

            // Log a cada 100ms (20 × 5ms)
            if (++cont_serial >= 20) {
                cont_serial = 0;
                Serial.printf("LQR  | Ang:%.3f w:%.2f Pos:%.3f v:%.2f u:%.1f V:%.1f\n",
                              theta, w, x, v, u_control, u_voltage);
            }
            break;
        }

        default:
            stopMotor();
            break;
    }

    // 6. Sincronismo (período fixo de 5ms)
    while (micros() - tempo_inicio < (unsigned long)(Ts * 1000000));
}

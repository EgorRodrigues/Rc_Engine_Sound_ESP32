# 📚 Guia de Estudo - RC Engine Sound Controller ESP32

Este guia vai te ajudar a entender **COMO** e **POR QUE** o código funciona, para que possamos refatorar com segurança.

---

## 🎯 Visão Geral do Sistema

### O Que Este Projeto Faz?

É um **controlador de som e luzes para veículos RC** (caminhões, escavadeiras, tanques) que:

1. **Lê sinais de um controle remoto** (SBUS, IBUS, PPM, PWM)
2. **Simula sons de motor** (aceleração, ré, turbo, etc.)
3. **Controla luzes** (faróis, setas, freio)
4. **Controla servos** (câmbio, direção, guincho)
5. **Gerencia transmissão** (marchas automáticas ou manuais)
6. **Protege bateria** (detecta voltagem e número de células)

### Hardware Típico

```
┌─────────────────────────────────────────────────────────────┐
│                     VEÍCULO RC                              │
│                                                             │
│  ┌──────────────┐     ┌──────────────┐     ┌────────────┐   │
│  │   ESP32      │────▶│  Amplificador│────▶│  Alto-     │   │
│  │   Board      │     │  PAM8403     │     │  falantes  │   │
│  │              │     └──────────────┘     └────────────┘   │
│  │  • Som       │                                           │
│  │  • Luzes     │     ┌──────────────┐     ┌────────────┐   │
│  │  • Servos    │────▶│  Servos &    │────▶│  Motor     │   │
│  │  • ESC       │     │  ESC         │     │  DC        │   │
│  └──────┬───────┘     └──────────────┘     └────────────┘   │
│         │                                                   │
│         │ Recebe sinal                                      │
│         ▼                                                   │
│  ┌──────────────┐                                           │
│  │ Receptor RC  │◀────── Controle Remoto                    │
│  │ (SBUS/IBUS)  │         (FlySky, FrSky, etc.)             │
│  └──────────────┘                                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 📂 Estrutura de Arquivos

```
Rc_Engine_Sound_ESP32/
├── src/
│   ├── src.ino              # ⭐ ARQUIVO PRINCIPAL (6549 linhas!)
│   │
│   ├── 0_GeneralSettings.h  # Configurações gerais
│   ├── 1_Vehicle.h          # Seleção do veículo
│   ├── 2_Remote.h           # Configuração do controle remoto
│   ├── 3_ESC.h              # Configuração do ESC (motor)
│   ├── 4_Transmission.h     # Configuração da transmissão
│   ├── 5_Shaker.h           # Motor vibrador (shaker)
│   ├── 6_Lights.h           # Configuração de luzes
│   ├── 7_Servos.h           # Configuração de servos
│   ├── 8_Sound.h            # Configuração de sons
│   ├── 9_Dashboard.h        # Display LCD (opcional)
│   ├── 10_Trailer.h         # Controle de trailer
│   │
│   ├── vehicles/            # Perfis de veículos
│   │   ├── sounds/          # Arquivos de som (.h com samples)
│   │   └── [veículos].h     # Configurações específicas
│   │
│   └── src/                 # Código auxiliar
│       ├── dashboard.cpp    # Display LCD
│       ├── SUMD.cpp         # Protocolo SUMD
│       ├── sbus.cpp         # Protocolo SBUS embarcado
│       ├── helper.h         # Funções utilitárias
│       └── curves.h         # Curvas de throttle
│
├── lib/                     # Bibliotecas externas
├── tools/                   # Ferramentas de conversão de áudio
└── platformio.ini           # Configuração do PlatformIO
```

---

## 🔄 Fluxo Principal do Programa

### 1. Setup (executado uma vez ao ligar)

```cpp
void setup() {
    // 1. Watchdog timers (previne travamentos)
    disableCore0WDT();
    
    // 2. Serial (debug)
    Serial.begin(115200);
    
    // 3. EEPROM (ler configurações salvas)
    setupEeprom();
    
    // 4. Bateria (detectar número de células)
    setupBattery();
    
    // 5. ESP-NOW (comunicação wireless com trailer)
    setupEspNow();
    
    // 6. Dashboard LCD (se habilitado)
    dashboard.init();
    
    // 7. Neopixel LEDs (se habilitado)
    setupNeopixel();
    
    // 8. Comunicação RC (SBUS, IBUS, PPM, PWM)
    #if defined SBUS_COMMUNICATION
        sBus.begin();
        setupMcpwm();  // Configurar PWM para servos
    #elif defined IBUS_COMMUNICATION
        iBus.begin();
        setupMcpwm();
    // ... etc
    #endif
    
    // 9. Timer de interrupção para som do motor
    variableTimer = timerBegin(0, 20, true);
    timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true);
    timerAlarmEnable(variableTimer);
    
    // 10. Timer de interrupção para sons fixos (buzina, etc.)
    fixedTimer = timerBegin(1, 20, true);
    timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true);
    timerAlarmEnable(fixedTimer);
    
    // 11. Task1 (roda no core 0 em paralelo)
    xTaskCreatePinnedToCore(
        Task1code,      // Função
        "Task1",        // Nome
        8192,           // Stack size
        NULL,           // Parâmetros
        1,              // Prioridade
        &Task1,         // Handle
        0               // Core 0
    );
}
```

### 2. Loop Principal (executa continuamente)

Na verdade, **NÃO há um `loop()` tradicional**! O código usa **tasks do FreeRTOS**:

```cpp
// Task1 roda no Core 0 em paralelo com o loop principal
void Task1code(void *parameters) {
    for(;;) {
        // 1. Ler sinais RC
        #if defined SBUS_COMMUNICATION
            readSbusCommands();
        #elif defined IBUS_COMMUNICATION
            readIbusCommands();
        // ... etc.
        #endif
        
        // 2. Processar canais (normalizar, inverter, etc.)
        processRawChannels();
        
        // 3. Controle do motor
        controlEngine();
        
        // 4. Controle da transmissão
        controlTransmission();
        
        // 5. Controle do ESC
        controlESC();
        
        // 6. Controle de luzes
        controlLights();
        
        // 7. Controle de servos
        mcpwmOutput();
        
        // 8. Interface serial (comandos USB)
        serialInterface();
        
        // 9. Interface web (configuração WiFi)
        webInterface();
        
        // 10. Dashboard LCD
        dashboard.update();
        
        delay(1);  // 1ms = 1000Hz loop rate
    }
}
```

---

## 🎛️ Sistema de Leitura do Controle Remoto

### Como Funciona a Leitura RC?

O sistema suporta **5 protocolos diferentes**, mas apenas **um por vez**:

```cpp
// Em 2_Remote.h, você escolhe UM protocolo:
#define SBUS_COMMUNICATION
// #define IBUS_COMMUNICATION
// #define PPM_COMMUNICATION
// #define SUMD_COMMUNICATION
// #define PWM_COMMUNICATION  // Padrão se nenhum for definido
```

### Fluxo de Leitura (Exemplo: SBUS)

```
Controle Remoto (FlySky)
         │
         │ Sinal SBUS (inverted UART 100000 baud)
         ▼
Receptor SBUS
         │
         │ Fio conectado no GPIO 36 (COMMAND_RX)
         ▼
ESP32 (Serial2)
         │
         │ sBus.read() lê 16 canais
         │ Valores: 172-1811 (unidades SBUS)
         ▼
Conversão para µs
         │
         │ map(172-1811, 1000-2000)
         │ Valores: 1000-2000 µs
         ▼
pulseWidthRaw[16]
         │
         │ processRawChannels()
         │ - Inverte canais se necessário
         │ - Aplica auto-zero
         │ - Suaviza (averaging)
         ▼
pulseWidth[16]
         │
         │ Valores finais prontos para uso
         │ Ex: pulseWidth[3] = 1500 (throttle neutro)
```

### Código de Leitura SBUS (simplificado)

```cpp
void readSbusCommands() {
    // Tenta ler pacote SBUS
    if (sBus.read(&SBUSchannels[0], &SBUSfailSafe, &SBUSlostFrame)) {
        sbusInit = true;
        lastSbusFailsafe = millis();
    }
    
    // Detecta failsafe por timeout
    if (millis() - lastSbusFailsafe > sbusFailsafeTimeout) {
        failSafe = true;
    }
    
    // Converte canais SBUS para pulse width (µs)
    if (!failSafe) {
        pulseWidthRaw[1] = map(SBUSchannels[STEERING - 1], 172, 1811, 1000, 2000);
        pulseWidthRaw[2] = map(SBUSchannels[GEARBOX - 1], 172, 1811, 1000, 2000);
        pulseWidthRaw[3] = map(SBUSchannels[THROTTLE - 1], 172, 1811, 1000, 2000);
        // ... etc. para 16 canais
    }
}
```

### Processamento dos Canais

```cpp
void processRawChannels() {
    for (uint8_t i = 1; i < PULSE_ARRAY_SIZE; i++) {
        
        // 1. Aplica curva exponencial (opcional)
        #ifdef EXPONENTIAL_THROTTLE
            if (i == 3) {  // Throttle apenas
                pulseWidthRaw2[i] = reMap(curveExponentialThrottle, pulseWidthRaw[i]);
            }
        #endif
        
        // 2. Inverte canal se configurado
        if (channelReversed[i]) {
            pulseWidthRaw3[i] = map(pulseWidthRaw2[i], 0, 3000, 3000, 0);
        } else {
            pulseWidthRaw3[i] = pulseWidthRaw2[i];
        }
        
        // 3. Auto-zero (calibra centro em 1500µs)
        if (channelAutoZero[i] && !autoZeroDone) {
            pulseOffset[i] = 1500 - pulseWidthRaw3[i];
        }
        pulseWidthRaw3[i] += pulseOffset[i];
        
        // 4. Limita valores válidos (1000-2000µs)
        pulseWidthRaw3[i] = constrain(pulseWidthRaw3[i], 1000, 2000);
        
        // 5. Suavização (opcional)
        #ifdef CHANNEL_AVERAGING
            smoothed[i] = (smoothed[i] * 3 + pulseWidthRaw3[i]) / 4;
            pulseWidth[i] = smoothed[i];
        #else
            pulseWidth[i] = pulseWidthRaw3[i];
        #endif
    }
}
```

---

## 🔊 Sistema de Som

### Arquitetura de Som

O sistema usa **2 timers de interrupção** que rodam em paralelo:

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32 CPU                            │
│                                                         │
│  Core 0: Task1 (controle geral)                         │
│  Core 1: Loop principal + Interrupts                    │
│                                                         │
│  ┌─────────────────┐    ┌─────────────────┐            │
│  │ variableTimer   │    │ fixedTimer      │            │
│  │ (Engine sound)  │    │ (Horn, etc.)    │            │
│  │                 │    │                 │            │
│  │ Sample rate:    │    │ Sample rate:    │            │
│  │ Variável        │    │ Fixo            │            │
│  │ (2000-8000 Hz)  │    │ (8000-48000 Hz) │            │
│  │                 │    │                 │            │
│  │ RPM do motor    │    │ Sons fixos:     │            │
│  │ controla speed  │    │ - Buzina        │            │
│  │                 │    │ - Sirene        │            │
│  │ Sons:           │    │ - Ré            │            │
│  │ - Idle          │    │ - Indicador     │            │
│  │ - Rev           │    │ - Wastegate     │            │
│  │ - Turbo         │    │ - Air brake     │            │
│  │ - Fan           │    │ - Shifting      │            │
│  │ - Jake brake    │    │ - Diesel knock  │            │
│  └────────┬────────┘    └────────┬────────┘            │
│           │                     │                      │
│           └──────────┬──────────┘                      │
│                      ▼                                  │
│              ┌───────────────┐                         │
│              │ DAC1 + DAC2   │                         │
│              │ (Audio out)   │                         │
│              └───────┬───────┘                         │
│                      │                                  │
│                      ▼                                  │
│              ┌───────────────┐                         │
│              │ Amplificador  │                         │
│              │ PAM8403       │                         │
│              └───────┬───────┘                         │
│                      │                                  │
│                      ▼                                  │
│              ┌───────────────┐                         │
│              │ Alto-falantes │                         │
│              └───────────────┘                         │
└─────────────────────────────────────────────────────────┘
```

### Timer de Velocidade Variável (Motor)

```cpp
// Interrupt que toca o som do motor
void IRAM_ATTR variablePlaybackTimer() {
    
    switch (engineState) {
        
        case OFF:
            // Motor desligado
            variableTimerTicks = 4000000 / startSampleRate;
            a = 0;  // Volume zero
            if (engineOn) {
                engineState = STARTING;
            }
            break;
            
        case STARTING:
            // Toca sample de partida
            variableTimerTicks = 4000000 / startSampleRate;
            if (curStartSample < startSampleCount - 1) {
                a = startSamples[curStartSample] * volume / 100;
                curStartSample++;
            } else {
                engineState = RUNNING;
            }
            break;
            
        case RUNNING:
            // Motor funcionando
            // RPM controla sample rate (pitch do som)
            variableTimerTicks = engineSampleRate;  // Varia com RPM!
            
            // Mix de idle + rev sounds
            a1 = samples[curEngineSample] * idleVolume / 100;
            a2 = revSamples[curRevSample] * revVolume / 100;
            a = a1 + a2;
            
            // Turbo sound
            c = turboSamples[curTurboSample] * turboVolume / 100;
            
            // Fan sound
            d = fanSamples[curFanSample] * fanVolume / 100;
            
            break;
            
        case STOPPING:
            // Desligando motor (fade out)
            variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100;
            a = samples[curEngineSample] * volume / attenuator;
            break;
    }
    
    // Output para DAC
    dacWrite(DAC1, constrain(a + c + d + dacOffset, 0, 255));
}
```

### Timer de Velocidade Fixa (Sons Diversos)

```cpp
// Interrupt que toca sons fixos (buzina, sirene, etc.)
void IRAM_ATTR fixedPlaybackTimer() {
    
    // Buzina
    if (hornTrigger || hornLatch) {
        if (curHornSample < hornSampleCount - 1) {
            a1 = hornSamples[curHornSample] * hornVolume / 100;
            curHornSample++;
        } else {
            hornLatch = false;
        }
    }
    
    // Sirene
    if (sirenTrigger || sirenLatch) {
        if (curSirenSample < sirenSampleCount - 1) {
            a2 = sirenSamples[curSirenSample] * sirenVolume / 100;
            curSirenSample++;
        } else {
            sirenLatch = false;
        }
    }
    
    // Som de ré
    if (engineRunning && escInReverse) {
        b1 = reversingSamples[curReversingSample] * reversingVolume / 100;
    }
    
    // Diesel knock (ignição)
    if (dieselKnockTrigger) {
        b7 = knockSamples[curDieselKnockSample] * knockVolume / 100;
    }
    
    // Mix de todos os sons
    a = a1 + a2;  // Buzina + sirene
    b = b1 + b7;  // Ré + knock
    
    // Output para DAC2
    dacWrite(DAC2, constrain(a + b + dacOffset, 0, 255));
}
```

### Samples de Som

Os sons são armazenados como **arrays de bytes** em arquivos `.h`:

```cpp
// Exemplo: CAT730Idle.h
const unsigned int sampleCount = 3280;
const signed char samples[] = {
    0, 12, 24, 36, 48, 60, ...  // 3280 samples
};

// Exemplo: CarHorn.h
const unsigned int hornSampleCount = 13513;
const signed char hornSamples[] = {
    0, -5, 10, -15, 20, -25, ...  // 13513 samples
};
```

**Como são criados:**
1. Grava som real (ex: motor V8)
2. Converte para .wav (8-bit, mono, 8000-48000 Hz)
3. Usa conversor online para gerar array C++
4. Inclui no arquivo de veículo

---

## 💡 Sistema de Luzes

### Controle de Luzes

```cpp
// Luzes são controladas por objetos statusLED
statusLED headLight(false);    // Farol
statusLED tailLight(false);    // Luz traseira
statusLED indicatorL(false);   // Seta esquerda
statusLED indicatorR(false);   // Seta direita
statusLED brakeLight(false);   // Luz de freio
statusLED reversingLight(false); // Luz de ré
statusLED beaconLight1(false); // Giroflex azul 1
statusLED beaconLight2(false); // Giroflex azul 2

// No loop:
void controlLights() {
    
    // Farol (liga com motor ou CH5)
    #ifdef AUTO_LIGHTS
        if (engineRunning) {
            headLight.on();
        } else {
            headLight.off();
        }
    #else
        if (pulseWidth[5] > 1600) {
            headLight.on();
        } else {
            headLight.off();
        }
    #endif
    
    // Setas (indicadores)
    if (indicatorLTrigger.justPressed()) {
        indicatorL.flash(200, 200);  // ON, OFF, repeat
    }
    if (indicatorRTrigger.justPressed()) {
        indicatorR.flash(200, 200);
    }
    
    // Luz de freio (ESC braking ou CH2)
    if (escIsBraking || pulseWidth[2] < 1400) {
        brakeLight.on();
    } else {
        brakeLight.off();
    }
    
    // Luz de ré (ESC reverse)
    if (escInReverse) {
        reversingLight.flash(100, 100);  // Pisca
    } else {
        reversingLight.off();
    }
    
    // Giroflex (CH4)
    if (blueLightTrigger) {
        beaconLight1.flash(100, 100);
        beaconLight2.flash(100, 100, 50);  // 50ms delay
    }
}
```

### Objeto statusLED

```cpp
// Biblioteca statusLED (https://github.com/TheDIYGuy999/statusLED)
class statusLED {
public:
    void begin(pin, timer, frequency);
    void on();
    void off();
    void flash(onTime, offTime, pause=0, pulses=0);
    void brightness(level);  // 0-255
    
private:
    // Usa hardware LEDC (PWM hardware ESP32)
    // Não usa CPU no loop!
};
```

---

## ⚙️ Sistema de Transmissão

### Transmissão Manual (3 marchas)

```cpp
void controlTransmission() {
    
    // Detecta posição do switch (CH2)
    if (pulseWidth[2] < 1400) {
        selectedGear = 1;  // Marcha 1
    } else if (pulseWidth[2] > 1600) {
        selectedGear = 3;  // Marcha 3
    } else {
        selectedGear = 2;  // Marcha 2
    }
    
    // Move servo do câmbio
    if (selectedGear == 1) {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, CH2L);
    } else if (selectedGear == 2) {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, CH2C);
    } else {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, CH2R);
    }
    
    // Som de shifting
    if (selectedGear != lastGear && !gearShiftingInProgress) {
        shiftingTrigger = true;
        gearShiftingInProgress = true;
        delay(500);  // Aguarda servo mover
        gearShiftingInProgress = false;
    }
}
```

### Transmissão Automática

```cpp
void controlAutomaticTransmission() {
    
    // Muda marcha baseado no RPM
    if (currentRpm > 400 && selectedAutomaticGear < 6) {
        selectedAutomaticGear++;  // Sobe marcha
        shiftingTrigger = true;
    }
    if (currentRpm < 200 && selectedAutomaticGear > 1) {
        selectedAutomaticGear--;  // Desce marcha
        shiftingTrigger = true;
    }
    
    // Limita RPM por marcha
    switch (selectedAutomaticGear) {
        case 1: speedLimit = 100; break;
        case 2: speedLimit = 200; break;
        case 3: speedLimit = 300; break;
        case 4: speedLimit = 400; break;
        case 5: speedLimit = 450; break;
        case 6: speedLimit = 500; break;
    }
}
```

---

## 🏎️ Controle do Motor (ESC)

### Como o ESC é Controlado

```cpp
void controlESC() {
    
    // Throttle do controle (0-500)
    int16_t requestedThrottle = 0;
    
    if (pulseWidth[3] > 1500) {
        // Acelerando
        requestedThrottle = map(pulseWidth[3], 1500, 2000, 0, 500);
    } else if (pulseWidth[3] < 1500) {
        // Freando/Ré
        requestedThrottle = map(pulseWidth[3], 1500, 1000, 0, -500);
    }
    
    // Inércia do motor (fade in/out)
    if (currentThrottle < requestedThrottle) {
        currentThrottle += accelerationRate;
    } else if (currentThrottle > requestedThrottle) {
        currentThrottle -= decelerationRate;
    }
    
    // Converte throttle para PWM do ESC (1000-2000µs)
    uint16_t escPulseWidth;
    
    if (currentThrottle > 0) {
        // Frente: 1500-2000µs
        escPulseWidth = map(currentThrottle, 0, 500, 1500, 2000);
    } else if (currentThrottle < 0) {
        // Ré: 1000-1500µs
        escPulseWidth = map(currentThrottle, -500, 0, 1000, 1500);
    } else {
        // Neutro: 1500µs
        escPulseWidth = 1500;
    }
    
    // Envia PWM para ESC (via MCPWM)
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, escPulseWidth);
    
    // Detecta estado do ESC
    escIsBraking = (currentThrottle < 0);
    escInReverse = (escPulseWidth < 1500);
}
```

### Motor com Inércia (RZ7886)

```cpp
// Para motor driver RZ7886 (controle bidirecional)
void controlRZ7886() {
    
    // Throttle (-500 a +500)
    int16_t throttle = currentThrottle;
    
    if (throttle > 0) {
        // Frente: PWM no pin 1, pin 2 desligado
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1500 + throttle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
    } else if (throttle < 0) {
        // Ré: PWM no pin 2, pin 1 desligado
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, 1500 + abs(throttle));
    } else {
        // Freio: ambos os pins em 1500
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
    }
}
```

---

## 🔋 Sistema de Bateria

### Detecção de Células

```cpp
void setupBattery() {
    #if defined BATTERY_PROTECTION
    
    // Lê voltagem da bateria (divisor resistivo)
    float voltage = batteryVolts();
    
    // Detecta número de células (2S, 3S, 4S LiPo)
    if (voltage <= 7.4) {
        numberOfCells = 2;  // 2S
    } else if (voltage <= 11.1) {
        numberOfCells = 3;  // 3S
    } else if (voltage <= 14.8) {
        numberOfCells = 4;  // 4S
    }
    
    // Toca beeps indicando número de células
    for (uint8_t i = 0; i < numberOfCells; i++) {
        tone(26, 3000, 4);  // BEEP
        delay(200);
    }
    
    // Verifica se bateria está descarregada
    if (voltage < numberOfCells * 3.5) {
        // Bateria fraca!
        outOfFuelMessageTrigger = true;
        
        // Pisca luzes 2x rápido
        for (int i = 0; i < 2; i++) {
            indicatorL.flash(70, 75, 500, 2);
            indicatorR.flash(70, 75, 500, 2);
        }
    }
    
    #endif
}

float batteryVolts() {
    // Lê ADC (0-4095)
    int adcValue = battery.read();
    
    // Converte para voltagem
    // Fórmula baseada no divisor resistivo
    float voltage = adcValue * (3.3 / 4095.0) * (RESISTOR_TO_BATTTERY_PLUS + RESISTOR_TO_GND) / RESISTOR_TO_GND;
    
    // Compensa queda do diodo
    voltage += DIODE_DROP;
    
    return voltage;
}
```

---

## 📡 Comunicação com Trailer (ESP-NOW)

### Envio de Dados para Trailer

```cpp
#if defined ENABLE_WIRELESS

// Estrutura de dados
typedef struct {
    uint8_t tailLight;
    uint8_t sideLight;
    uint8_t reversingLight;
    uint8_t indicatorL;
    uint8_t indicatorR;
    bool legsUp;
    bool legsDown;
    bool rampsUp;
    bool rampsDown;
    bool beaconsOn;
} struct_message;

struct_message trailerData;

void sendTrailerData() {
    
    // Copia estado das luzes
    trailerData.tailLight = tailLight.getState();
    trailerData.sideLight = sideLight.getState();
    trailerData.reversingLight = reversingLight.getState();
    trailerData.indicatorL = indicatorL.getState();
    trailerData.indicatorR = indicatorR.getState();
    
    // Envia para trailer 1
    if (useTrailer1) {
        esp_now_send(broadcastAddress1, (uint8_t*)&trailerData, sizeof(trailerData));
    }
    
    // Envia para trailer 2
    if (useTrailer2) {
        esp_now_send(broadcastAddress2, (uint8_t*)&trailerData, sizeof(trailerData));
    }
    
    // Envia para trailer 3
    if (useTrailer3) {
        esp_now_send(broadcastAddress3, (uint8_t*)&trailerData, sizeof(trailerData));
    }
}

#endif
```

---

## 🎮 Mapeamento de Canais

### Exemplo: FlySky FS-i6S

```cpp
// Em 2_Remote.h:
#ifdef FLYSKY_FS_I6S

// Mapeamento: Canal do controle -> Canal do sound controller
#define STEERING 1      // Controle CH1 -> Sound CH1 (direção)
#define GEARBOX 10      // Controle CH10 -> Sound CH2 (câmbio)
#define THROTTLE 3      // Controle CH3 -> Sound CH3 (acelerador)
#define HORN 6          // Controle CH6 -> Sound CH4 (buzina)
#define FUNCTION_R 2    // Controle CH2 -> Sound CH5 (funções direita)
#define FUNCTION_L 4    // Controle CH4 -> Sound CH6 (funções esquerda)

// Canais invertidos?
boolean channelReversed[17] = {
    false,  // CH0 (não usado)
    false,  // CH1 (direção: não inverte)
    false,  // CH2 (câmbio: não inverte)
    false,  // CH3 (throttle: não inverte)
    false,  // CH4 (buzina: não inverte)
    true,   // CH5 (funções: inverte!)
    false,  // CH6 (indicadores: não inverte)
    // ... etc.
};

// Auto-zero (calibra centro automaticamente)
boolean channelAutoZero[17] = {
    false,  // CH0
    true,   // CH1 (direção: tem mola, auto-zero OK)
    false,  // CH2 (câmbio: switch 3 posições, sem auto-zero)
    true,   // CH3 (throttle: tem mola, auto-zero OK)
    false,  // CH4 (buzina: switch, sem auto-zero)
    true,   // CH5 (funções: tem mola, auto-zero OK)
    true,   // CH6 (indicadores: tem mola, auto-zero OK)
    // ... etc.
};

#endif
```

---

## 🧠 Conceitos Importantes

### 1. FreeRTOS Tasks

O ESP32 tem **2 cores** e usa **FreeRTOS**:

```cpp
// Task1 roda no Core 0
xTaskCreatePinnedToCore(
    Task1code,    // Função
    "Task1",      // Nome
    8192,         // Stack (bytes)
    NULL,         // Parâmetros
    1,            // Prioridade (0-24)
    &Task1,       // Handle
    0             // Core (0 ou 1)
);

// Loop principal roda no Core 1
void loop() {
    // Roda em paralelo com Task1
}
```

### 2. Interrupt Timers

```cpp
// Timer 0: Som do motor (sample rate variável)
variableTimer = timerBegin(0, 20, true);  // Timer 0, prescaler 20
timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true);
timerAlarmWrite(variableTimer, variableTimerTicks, true);  // Auto-reload
timerAlarmEnable(variableTimer);

// Timer 1: Sons fixos (sample rate fixo)
fixedTimer = timerBegin(1, 20, true);
timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true);
timerAlarmEnable(fixedTimer);
```

### 3. MCPWM (Motor Control PWM)

```cpp
// Configura MCPWM para servos (50Hz)
mcpwm_config_t pwm_config;
pwm_config.frequency = 50;           // 50Hz para servos
pwm_config.cmpr_a = 0;               // Duty cycle inicial
pwm_config.counter_mode = MCPWM_UP_COUNTER;
pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

// Move servo para posição (1000-2000µs)
mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
```

### 4. EEPROM (Armazenamento)

```cpp
// Endereços na EEPROM
#define adr_eprom_init 0
#define adr_eprom_useTrailer1 4
#define adr_eprom_hazardsWhile5thWheelUnlocked 96
#define adr_eprom_esc_pulse_span 172
// ... etc.

// Escreve na EEPROM
EEPROM.write(adr_eprom_init, eeprom_id);
EEPROM.writeUShort(adr_eprom_esc_pulse_span, escPulseSpan);
EEPROM.commit();  // Importante!

// Lê da EEPROM
useTrailer1 = EEPROM.read(adr_eprom_useTrailer1);
escPulseSpan = EEPROM.readUShort(adr_eprom_esc_pulse_span);
```

---

## 📊 Estatísticas do Código

```
src.ino: 6549 linhas
├── Variáveis globais: ~500 linhas
├── Setup: ~600 linhas
├── Interrupts (ISRs): ~900 linhas
├── Leitura RC: ~600 linhas
├── Task1 (loop principal): ~2000 linhas
├── Controle de motor: ~500 linhas
├── Controle de luzes: ~300 linhas
├── Som: ~800 linhas
└── Auxiliares: ~350 linhas

Arquivos de configuração (.h): ~1200 linhas cada
Arquivos de som (.h): ~50-50000 linhas cada (samples!)
```

---

## 🎯 Pontos de Atenção para Refatoração

### 1. Dependências Fortes

```cpp
// src.ino depende de:
- 0_GeneralSettings.h
- 1_Vehicle.h (que inclui sons)
- 2_Remote.h (que define protocolo RC)
- 3_ESC.h
- 4_Transmission.h
- 5_Shaker.h
- 6_Lights.h
- 7_Servos.h
- 8_Sound.h
- 9_Dashboard.h
- 10_Trailer.h

// Sons dependem do veículo:
vehicles/CaterpillarD6Dozer.h
  └── vehicles/sounds/CAT730Idle.h
  └── vehicles/sounds/CAT730Start.h
  └── vehicles/sounds/CAT730Rev.h
  └── ...
```

### 2. Variáveis Globais

**MUITAS** variáveis globais (cerca de 300+):

```cpp
// Estado do motor
volatile boolean engineOn;
volatile boolean engineRunning;
volatile int32_t currentRpm;

// Controle RC
uint16_t pulseWidth[17];
boolean failSafe;

// Luzes
volatile boolean lightsOn;
volatile boolean headLightsHighBeamOn;

// Transmissão
uint8_t selectedGear;
boolean gearUpShiftingInProgress;

// ESC
volatile boolean escIsBraking;
uint16_t currentSpeed;

// ... e muitas mais!
```

### 3. Interrupts Não Podem Usar delay()

```cpp
// ERRADO (não funciona em interrupts):
void IRAM_ATTR myInterrupt() {
    delay(100);  // ❌ Não funciona!
}

// CERTO:
void IRAM_ATTR myInterrupt() {
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 100) {
        lastTime = millis();
        // Faz algo
    }
}
```

### 4. IRAM_ATTR

```cpp
// Funções de interrupt devem estar na IRAM:
void IRAM_ATTR variablePlaybackTimer() {
    // Código aqui é executado na RAM, não na flash
    // Mais rápido e não quebra se flash estiver ocupada
}
```

---

## ✅ Checklist de Entendimento

Antes de refatorar, certifique-se de entender:

- [ ] Como o sinal RC é lido e processado
- [ ] Como os timers de interrupt funcionam
- [ ] Como o som é gerado (DAC + samples)
- [ ] Como as luzes são controladas (statusLED)
- [ ] Como o ESC/motor é controlado (MCPWM)
- [ ] Como a transmissão funciona (servos)
- [ ] Como a bateria é monitorada (ADC)
- [ ] Como ESP-NOW comunica com trailers
- [ ] Quais são as dependências entre arquivos
- [ ] Quais variáveis são compartilhadas entre tasks

---

## 📚 Próximos Passos de Estudo

1. **Leia os arquivos de configuração** na ordem:
   - `0_GeneralSettings.h`
   - `2_Remote.h`
   - `1_Vehicle.h`

2. **Trace o fluxo de dados**:
   - Controle → Receptor → ESP32 → pulseWidth[] → processRawChannels() → controle do motor

3. **Entenda os interrupts**:
   - variablePlaybackTimer() (motor)
   - fixedPlaybackTimer() (sons fixos)

4. **Estude FreeRTOS no ESP32**:
   - Tasks
   - Semáforos
   - Mutexes

5. **Experimente**:
   - Adicione Serial.printf() para debug
   - Modifique um parâmetro e veja o que acontece
   - Crie um novo perfil de veículo

---

**Dúvidas?** Me pergunte sobre qualquer parte específica que não esteja clara!

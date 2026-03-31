# 📡 Guia de Estudo: 2_Remote.h

## O Que Este Arquivo Faz?

Configura **como o controle remoto se comunica** com o sound controller:

1. **Seleciona o protocolo** (SBUS, IBUS, PPM, PWM, etc.)
2. **Mapeia os canais** do controle para funções do veículo
3. **Configura inversão** e calibração de cada canal
4. **Define perfis** para diferentes controles comerciais

---

## 🏗️ Estrutura do Arquivo

```
2_Remote.h (1248 linhas)
│
├── Seleção do Perfil do Controle (linha 1-20)
│   └── Qual controle você tem?
│
├── Configuração da Placa (linha 22-23)
│   └── Versão do hardware
│
├── Protocolo de Comunicação (linha 25-45)
│   ├── SBUS_COMMUNICATION
│   ├── IBUS_COMMUNICATION
│   ├── SUMD_COMMUNICATION
│   ├── PPM_COMMUNICATION
│   └── PWM (padrão se nenhum definido)
│
├── Configurações de Canal (linha 47-55)
│   ├── EXPONENTIAL_THROTTLE
│   ├── EXPONENTIAL_STEERING
│   └── CHANNEL_AVERAGING
│
└── Perfis de Controle (linha 90+)
    ├── FLYSKY_FS_I6X
    ├── FLYSKY_FS_I6S
    ├── FLYSKY_FS_I6S_LOADER
    ├── FLYSKY_FS_I6S_DOZER
    ├── FLYSKY_FS_I6S_EXCAVATOR
    ├── FRSKY_TANDEM_EXCAVATOR
    ├── FRSKY_TANDEM_HARMONY_LOADER
    ├── FRSKY_TANDEM_CRANE
    ├── FLYSKY_GT5
    ├── RGT_EX86100
    ├── GRAUPNER_MZ_12
    └── MICRO_RC
```

---

## 🔧 Configurações Principais

### 1. Selecionar Seu Controle

```cpp
// Descomente APENAS UM perfil:
// #define FLYSKY_FS_I6X
#define FLYSKY_FS_I6S_DOZER  // ← Seu controle
// #define FRSKY_TANDEM_EXCAVATOR
```

### 2. Selecionar Protocolo

```cpp
// Descomente APENAS UM:
#define SBUS_COMMUNICATION      // ← Recomendado
// #define IBUS_COMMUNICATION
// #define PPM_COMMUNICATION
// #define PWM_COMMUNICATION     // ← Padrão se nenhum definido
```

### 3. Configurar Canais

Cada perfil define:

```cpp
#ifdef FLYSKY_FS_I6S_DOZER

// Mapeamento: Controle → Sound Controller
#define STEERING 1      // Controle CH1 → Direção
#define GEARBOX 7       // Controle CH7 → Câmbio
#define THROTTLE 9      // Controle CH9 → Acelerador
#define HORN 10         // Controle CH10 → Buzina
#define FUNCTION_R 3    // Controle CH3 → Funções direita
#define FUNCTION_L 1    // Controle CH1 → Funções esquerda
#define POT2 2          // Controle CH2 → Potenciômetro 2
#define MODE1 6         // Controle CH6 → Modo 1
#define MODE2 8         // Controle CH8 → Modo 2

// Inversão de canal (true = inverte direção)
boolean channelReversed[17] = {
    false, false, false, ... // Um por canal (0-16)
};

// Auto-zero (true = calibra centro automaticamente)
boolean channelAutoZero[17] = {
    false, true, true, ... // Só usa em canais com mola
};

// Faixa do sinal
const uint16_t pulseNeutral = 30;   // Faixa morta: 1500 ± 30µs
const uint16_t pulseSpan = 480;     // Curso total: ~480µs

#endif
```

---

## 📊 Entendendo os Canais

### Canais do Sound Controller

| Canal | Função | Tipo | Auto-Zero? |
|-------|--------|------|------------|
| CH1 | Direção / Bucket | Proporcional | ✅ Sim |
| CH2 | Câmbio / Dipper | 3-pos ou Prop. | ❌ Não (switch) |
| CH3 | Acelerador / Throttle | Proporcional | ✅ Sim |
| CH4 | Buzina / Horn | Switch | ❌ Não |
| CH5 | Boom / Função R | Proporcional | ✅ Sim |
| CH6 | Esteira Esq. / Função L | Proporcional | ✅ Sim |
| CH7 | Esteira Dir. / Pot2 | Proporcional | ✅ Sim |
| CH8 | Swing / Modo 1 | Switch/Prop. | ❌ Não |
| CH9 | Luzes / Modo 2 | Switch | ❌ Não |
| CH10 | ISO/SAE / Momentary | Switch | ❌ Não |

### Quando Usar Auto-Zero

```cpp
// ✅ USA auto-zero (canais com mola, centro em 1500µs):
boolean channelAutoZero[17] = {
    ...,
    true,  // CH1 Steering (volta ao centro)
    true,  // CH3 Throttle (volta ao centro)
    true,  // CH5 Function R (volta ao centro)
    ...
};

// ❌ NÃO usa auto-zero (switches, potenciômetros):
boolean channelAutoZero[17] = {
    ...,
    false,  // CH2 Gearbox (switch 3 posições)
    false,  // CH4 Horn (switch)
    false,  // CH7 Pot2 (potenciômetro)
    ...
};
```

---

## 🔄 Protocolos de Comunicação

### SBUS (Recomendado)

```cpp
#define SBUS_COMMUNICATION
uint32_t sbusBaud = 100000;        // Baud rate
#define EMBEDDED_SBUS              // Usa código embarcado (não biblioteca)
uint16_t sbusFailsafeTimeout = 100; // Failsafe após 100ms sem sinal
boolean sbusInverted = true;       // SBUS padrão é invertido
```

**Vantagens:**
- 16 canais
- Failsafe detectado
- Apenas 1 fio (RX)
- Mais confiável

**Conexão:**
```
Receptor SBUS → ESP32
    TX  ──────── GPIO 36 (COMMAND_RX)
    GND ──────── GND
```

### IBUS

```cpp
#define IBUS_COMMUNICATION
```

**Vantagens:**
- 13 canais
- Compatível com FlySky antigos

**Desvantagens:**
- ❌ Sem failsafe hardware
- Mau contato = crash

### PPM

```cpp
#define PPM_COMMUNICATION
```

**Vantagens:**
- 8 canais
- 1 fio apenas

**Desvantagens:**
- Canais jittery (instáveis)
- Menos confiável

### PWM (Padrão)

```cpp
// Nenhum protocolo definido = PWM automático
```

**Vantagens:**
- Simples
- Cada canal é independente

**Desvantagens:**
- Só 6 canais
- Muitos fios (6 sinais + GND)

**Conexão:**
```
Receptor PWM → ESP32
    CH1 ──────── GPIO 13
    CH2 ──────── GPIO 12
    CH3 ──────── GPIO 14
    CH4 ──────── GPIO 27
    CH5 ──────── GPIO 35
    CH6 ──────── GPIO 34
    GND ──────── GND
```

---

## ⚙️ Configurações Avançadas

### Curva Exponencial

```cpp
// Throttle exponencial (controle fino em baixa velocidade)
#define EXPONENTIAL_THROTTLE

// Direção exponencial (mais precisão no centro)
#define EXPONENTIAL_STEERING
```

### Suavização de Canais

```cpp
// Média móvel para canais instáveis
#define CHANNEL_AVERAGING
```

### Modos Automáticos

```cpp
// Luzes ligam com motor
#define AUTO_LIGHTS

// Motor liga/desliga com throttle
#define AUTO_ENGINE_ON_OFF

// Setas automáticas por ângulo de direção
#define AUTO_INDICATORS
```

---

## 🎮 Exemplo: Configurando Seu Controle

### Passo 1: Identificar Seu Controle

Qual controle você tem?
- FlySky FS-i6, FS-i6X, FS-i6S?
- FrSky Taranis, Tandem XE?
- Graupner MZ?
- Outro?

### Passo 2: Selecionar Perfil

```cpp
// Exemplo: FlySky FS-i6S
#define FLYSKY_FS_I6S
```

### Passo 3: Selecionar Protocolo

```cpp
// Se seu receptor for SBUS:
#define SBUS_COMMUNICATION
boolean sbusInverted = true;

// Se seu receptor for IBUS:
#define IBUS_COMMUNICATION
```

### Passo 4: Configurar Canais

No seu controle, configure:
- CH1: Steering (direção)
- CH2: Gearbox (câmbio, switch 3 posições)
- CH3: Throttle (acelerador)
- CH4: Horn (buzina)
- CH5: Function R (funções direita)
- CH6: Function L (funções esquerda)

### Passo 5: Ajustar Inversão

Teste cada canal. Se estiver invertido:

```cpp
boolean channelReversed[17] = {
    ...,
    true,  // Mude para true se canal estiver ao contrário
    ...
};
```

### Passo 6: Testar

1. Ligue o controle
2. Ligue o sound controller
3. LEDs indicam status:
   - ✅ 1 flash longo = OK
   - ❌ 3 flashes curtos = Sem sinal RC
   - ❌ N flashes = Canal N fora de faixa

---

## 📝 Apêndice: Adicionando Bluetooth

### Visão Geral

Bluetooth permite usar **smartphone como controle** via app.

### Hardware Necessário

```
ESP32 (já tem Bluetooth)
   │
   └─ Não precisa hardware extra!
```

### Passo 1: Ativar Bluetooth no 2_Remote.h

```cpp
// Adicione no início do arquivo:
#define ENABLE_BLUETOOTH

// Comente outros protocolos:
// #define SBUS_COMMUNICATION
// #define IBUS_COMMUNICATION
```

### Passo 2: Criar BluetoothInput

Crie `src/rc_input/BluetoothInput.h`:

```cpp
#ifndef BLUETOOTH_INPUT_H
#define BLUETOOTH_INPUT_H

#include "RCInput.h"
#include <BluetoothSerial.h>

class BluetoothInput : public RCInput {
public:
    bool begin(int rxPin = 0, int txPin = 0) override;
    bool read() override;
    uint16_t getChannel(uint8_t channel) override;
    bool isFailsafe() override { return _failsafe; }
    const char* getProtocolName() override { return "Bluetooth"; }
    
private:
    BluetoothSerial _bt;
    unsigned long _lastReadTime = 0;
    uint16_t _failsafeTimeout = 300;
};

#endif
```

### Passo 3: Implementar BluetoothInput.cpp

```cpp
#include "BluetoothInput.h"

bool BluetoothInput::begin(int rxPin, int txPin) {
    _bt.begin("RC_Engine_Sound");  // Nome do dispositivo
    Serial.println("Bluetooth: Aguardando pareamento...");
    _ready = true;
    return true;
}

bool BluetoothInput::read() {
    if (!_ready) return false;
    
    // Lê dados do smartphone
    if (_bt.available()) {
        // Formato esperado: "CH1,CH2,CH3,CH4,CH5,CH6"
        String data = _bt.readStringUntil('\n');
        
        // Parse dos canais
        int ch = 1;
        int pos = 0;
        while (ch < 7 && pos < data.length()) {
            int comma = data.indexOf(',', pos);
            if (comma == -1) comma = data.length();
            
            String value = data.substring(pos, comma);
            _channels[ch] = value.toInt();
            
            pos = comma + 1;
            ch++;
        }
        
        _lastReadTime = millis();
        _failsafe = false;
        return true;
    }
    
    // Timeout = failsafe
    if (millis() - _lastReadTime > _failsafeTimeout) {
        _failsafe = true;
    }
    
    return false;
}
```

### Passo 4: Registrar no RCInputManager

Em `RCInputManager.h`:

```cpp
#ifdef ENABLE_BLUETOOTH
class BluetoothInput;
#endif
```

Em `RCInputManager.cpp`:

```cpp
#ifdef ENABLE_BLUETOOTH
#include "BluetoothInput.h"
BluetoothInput* _bluetoothInput = nullptr;
#endif

// No begin():
#elif defined ENABLE_BLUETOOTH
    _bluetoothInput = new BluetoothInput();
    _activeInput = _bluetoothInput;
```

### Passo 5: App de Smartphone

**Android:**
- Baixe "Arduino Bluetooth RC Car"
- Ou crie app personalizado com MIT App Inventor

**Configurar App:**
- Conecte no "RC_Engine_Sound"
- Envie dados no formato: `1500,1500,1500,1500,1500,1500\n`
- Atualize a 50Hz (20ms)

### Passo 6: Testar

```cpp
void setup() {
    rcManager.begin();
    
    Serial.println("Pareie com: RC_Engine_Sound");
    Serial.println("PIN: 1234 (padrão)");
}

void loop() {
    if (rcManager.read()) {
        Serial.print("CH1: ");
        Serial.println(rcManager.getChannel(1));
    }
}
```

### Limitações do Bluetooth

| Característica | Valor |
|----------------|-------|
| Alcance | 10-30m |
| Latência | 50-100ms |
| Canais | 6+ |
| Bateria | Drena smartphone |

---

## 📡 Apêndice: Adicionando ESP-NOW

### Visão Geral

ESP-NOW permite **comunicação wireless ESP32↔ESP32** sem WiFi.

### Hardware Necessário

```
Transmissor (Controle)          Receptor (Veículo)
┌─────────────────┐            ┌─────────────────┐
│  ESP32 DevKit   │            │  Sound Controller│
│                 │            │  (ESP32)         │
│  A0 ─── Joystick│            │                  │
│  A1 ─── Joystick│◄──220m───►│  GPIO 36 (RX)    │
│  A2 ─── Switch  │   ESP-NOW │                  │
│  A3 ─── Switch  │            │                  │
│  GND ───────────│            │  GND             │
│  5V ────────────│            │  5V              │
└─────────────────┘            └─────────────────┘
```

### Passo 1: Ativar ESP-NOW no 2_Remote.h

```cpp
// Adicione:
#define ENABLE_ESP_NOW

// Comente outros:
// #define SBUS_COMMUNICATION
```

### Passo 2: Criar ESPNowInput

Já existe em `src/rc_input/ESPNowInput.h`

### Passo 3: Código do Transmissor

Carregue este código no **ESP32 do controle**:

```cpp
/*
 * Transmissor ESP-NOW para RC Engine Sound
 * Carregar no ESP32 do controle remoto
 */

#include <esp_now.h>
#include <WiFi.h>

// MAC do receptor (sound controller)
// Descubra no Serial Monitor do receptor:
uint8_t receiverMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Estrutura de dados (16 canais)
typedef struct {
    uint16_t channels[16];
    uint8_t id;
} DataPacket;

DataPacket data;

// Callback de envio
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Enviado com sucesso!");
    } else {
        Serial.println("Erro no envio!");
    }
}

void setup() {
    Serial.begin(115200);
    
    // Inicializa WiFi (necessário para ESP-NOW)
    WiFi.mode(WIFI_STA);
    
    // Inicializa ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init falhou!");
        return;
    }
    
    // Registra callback
    esp_now_register_send_cb(OnDataSent);
    
    // Adiciona peer (receptor)
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Falha ao adicionar peer!");
        return;
    }
    
    Serial.println("Transmissor ESP-NOW pronto!");
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    
    // Inicializa canais em neutro
    for (int i = 0; i < 16; i++) {
        data.channels[i] = 1500;
    }
}

void loop() {
    // Lê joysticks e switches
    data.channels[0] = map(analogRead(A0), 0, 4095, 1000, 2000); // Steering
    data.channels[1] = map(analogRead(A1), 0, 4095, 1000, 2000); // Throttle
    data.channels[2] = map(analogRead(A2), 0, 4095, 1000, 2000); // Auxiliar 1
    data.channels[3] = map(analogRead(A3), 0, 4095, 1000, 2000); // Auxiliar 2
    
    // Botões (1000 = off, 2000 = on)
    if (digitalRead(4) == HIGH) {
        data.channels[4] = 2000;  // Horn
    } else {
        data.channels[4] = 1500;
    }
    
    // Envia dados
    esp_now_send(receiverMAC, (uint8_t*)&data, sizeof(data));
    
    delay(20);  // 50Hz (20ms)
}
```

### Passo 4: Descobrir MAC do Transmissor

1. Carregue código acima no ESP32 transmissor
2. Abra Serial Monitor (115200)
3. Anote MAC exibido: `AA:BB:CC:DD:EE:FF`
4. No receptor (sound controller), configure:

```cpp
// Em 2_Remote.h ou RCInputManager.cpp:
uint8_t espNowTransmitterMAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```

### Passo 5: Integrar com RCInputManager

Em `RCInputManager.cpp`:

```cpp
#ifdef ENABLE_ESP_NOW
#include "ESPNowInput.h"
ESPNowInput* _espNowInput = nullptr;
#endif

bool RCInputManager::begin(int rxPin, int txPin) {
    #ifdef ENABLE_ESP_NOW
        _espNowInput = new ESPNowInput();
        _activeInput = _espNowInput;
        
        // Adiciona peer transmissor
        _espNowInput->addPeer(espNowTransmitterMAC);
    #endif
    // ...
}
```

### Passo 6: Testar

```
1. Ligue transmissor (controle)
2. Ligue receptor (veículo)
3. Serial do receptor mostra:
   "ESP-NOW: Peer added: AA:BB:CC:DD:EE:FF"
   "ESP-NOW: Data received"
4. Mova joysticks → dados aparecem no serial
```

### Vantagens do ESP-NOW

| Característica | Valor |
|----------------|-------|
| Alcance | ~220m (campo aberto) |
| Latência | <10ms |
| Canais | 16 |
| Custo | ~$5 (ESP32) |
| Bateria | Baixo consumo |

### Desvantagens

- Precisa programar ESP32 transmissor
- Alcance menor que SBUS (2.4GHz)
- Paredes/obstáculos reduzem alcance

---

## ✅ Checklist de Configuração

### Para Controle Comercial (SBUS/IBUS)

- [ ] Selecionar perfil do controle
- [ ] Selecionar protocolo
- [ ] Configurar `sbusInverted` (se SBUS)
- [ ] Testar todos os canais
- [ ] Ajustar `channelReversed` se necessário

### Para ESP-NOW

- [ ] Criar transmissor com ESP32
- [ ] Carregar código transmissor
- [ ] Descobrir MAC do transmissor
- [ ] Configurar MAC no receptor
- [ ] Ativar `ENABLE_ESP_NOW`
- [ ] Testar comunicação

### Para Bluetooth

- [ ] Ativar `ENABLE_BLUETOOTH`
- [ ] Criar BluetoothInput
- [ ] Instalar app no smartphone
- [ ] Parear dispositivo
- [ ] Testar controle

---

## 🔍 Debug de Problemas

### Sem Sinal RC (3 flashes LED)

```cpp
// Verifique:
1. Controle ligado?
2. Receptor ligado?
3. Fio RX conectado no GPIO 36?
4. Protocolo correto definido?
5. sbusInverted correto?
```

### Canal Invertido

```cpp
// No perfil do controle:
boolean channelReversed[17] = {
    ...,
    true,  // Inverte este canal
    ...
};
```

### Canal Fora de Faixa (N flashes)

```cpp
// Ajuste no controle:
1. Trim do canal para centro
2. Dual Rate para 100%
3. End Points para 100%
```

### Failsafe Constante

```cpp
// Aumente timeout:
uint16_t sbusFailsafeTimeout = 200;  // De 100 para 200ms
```

---

**Dúvidas?** Consulte o `STUDY_GUIDE.md` para conceitos gerais ou me pergunte sobre algo específico!

# Glouse: Mouse BLE Controlado por Movimento

[Read in English 🇺🇸](./README.en.md)

Glouse é um mouse BLE (Bluetooth Low Energy) controlado por movimento, utilizando o microcontrolador ESP32 e o sensor de movimento MPU6050. Ele permite controlar o cursor e realizar ações do mouse usando gestos das mãos e botões sensíveis ao toque integrados em uma luva.

## Índice

1. [Recursos](#recursos)
2. [Componentes Utilizados](#componentes-utilizados)
   - [Hardware](#hardware)
   - [Ambiente de Software](#ambiente-de-software)
   - [Bibliotecas](#bibliotecas)
3. [Como Funciona](#como-funciona)
4. [Configuração de Pinos](#configuração-de-pinos)
5. [Primeiros Passos](#primeiros-passos)
   - [Pré-requisitos](#pré-requisitos)
   - [Configuração](#configuração)
   - [Uso](#uso)
6. [Observações](#observações)
7. [Licença](#licença)

## Recursos

- **Controle por Movimento**: Use o MPU6050 para mover o ponteiro do mouse com base no pitch e roll.
- **Botões Sensíveis ao Toque**: Realize cliques e outras ações do mouse com entradas sensíveis ao toque.
- **Conectividade BLE**: Conecte o ESP32 ao computador ou dispositivo como um mouse Bluetooth.
- **Sensibilidade Personalizável**: Ajuste a sensibilidade do ponteiro e da rolagem.
- **Integração com FreeRTOS**: Aproveita as capacidades multitarefa do ESP32.

## Componentes Utilizados

### Hardware

- Microcontrolador ESP32
- Sensor de movimento MPU6050 (acelerômetro e giroscópio de 6 eixos)
- Entradas sensíveis ao toque (pinos de toque do ESP32)

### Ambiente de Software

- PlatformIO no Visual Studio Code
- Framework Arduino

### Bibliotecas

- [I2Cdev](https://github.com/jrowberg/i2cdevlib): Para comunicação I2C com o MPU6050.
- [MPU6050\_6Axis\_MotionApps20](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050): Simplifica a interação com o MPU6050.
- [Wire](https://www.arduino.cc/en/Reference/Wire): Biblioteca I2C para Arduino.
- [BleMouse](https://github.com/T-vK/ESP32-BLE-Mouse): Permite que o ESP32 atue como um mouse BLE.

## Como Funciona

1. **Detecção de Movimento**:

   - O MPU6050 captura dados de movimento (yaw, pitch e roll).
   - Os dados são processados para controlar o movimento do ponteiro do mouse.

2. **Controles por Toque**:

   - Pinos sensíveis ao toque no ESP32 detectam interações do usuário.
   - Cada pino corresponde a uma ação específica do mouse (ex.: clique esquerdo, clique direito, rolagem).

3. **Funcionalidade de Mouse BLE**:

   - O ESP32 atua como um mouse BLE e se comunica com um dispositivo host.

## Configuração de Pinos

| Pino ESP32 | Conexão    | Local   | Função                                   |
| ---------- | ---------- | ------- | ---------------------------------------- |
| 21         | SDA        | MPU6050 | Leitura de dados do MPU                  |
| 22         | SCL        | MPU6050 | Clock do MPU                             |
| 3.3V       | 3.3V       | MPU6050 | Alimentação do MPU                       |
| GND        | GND        | MPU6050 | Terra do MPU                             |
| GND        | Sua mão    |         | Terra para estabilidade das leituras*    |
| 15         | Dedo 1.1   | Luva    | Clique Esquerdo                          |
| 13         | Dedo 1.2   | Luva    | Voltar                                   |
| 12         | Dedo 1.3   | Luva    | Avançar                                  |
| 14         | Dedo 2.1   | Luva    | Clique Direito                           |
| 27         | Dedo 2.2   | Luva    | Clique do Meio                           |
| 33         | Dedo 3.1   | Luva    | Rolagem (especial)                       |
| 32         | Dedo 3.2   | Luva    | Configuração (especial)                  |

**\*** O pino GND conectado à sua mão melhora a sensibilidade e estabilidade das leituras de toque, fornecendo uma referência de terra consistente.

## Primeiros Passos

### Pré-requisitos

- Instale o [PlatformIO](https://platformio.org/) no Visual Studio Code.
- Certifique-se de ter as bibliotecas necessárias instaladas (listadas acima ou no arquivo platformio.ini).

### Configuração

1. Clone o repositório.
2. Abra o projeto no VS Code com o PlatformIO.
3. Conecte o ESP32 ao computador via USB.
4. Compile e carregue o código no ESP32.

### Uso

- Emparelhe o ESP32 com seu computador ou dispositivo via Bluetooth (nome padrão do dispositivo: "Glouse").
- Use a luva e os controles sensíveis ao toque para manipular o mouse com movimentos.

## Observações

- Este projeto está em desenvolvimento, e alguns recursos ou detalhes de fiação podem mudar no futuro.
- O diagrama de fiação será adicionado em uma atualização futura.

## Licença

Este projeto é open-source e está disponível sob a Licença MIT.

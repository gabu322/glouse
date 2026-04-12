![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=cplusplus&logoColor=white)
![Plataforma](https://img.shields.io/badge/Plataforma-ESP32-E7352C?style=flat)
![Protocolo](https://img.shields.io/badge/Protocolo-BLE_HID-0082FC?style=flat)
![Licença](https://img.shields.io/badge/Licença-MIT-22C55E?style=flat)
![Status](https://img.shields.io/badge/Status-Em_Desenvolvimento-F59E0B?style=flat)

[Read in English 🇺🇸](README.md)

# Glouse: Mouse BLE Controlado por Movimento

Glouse é uma luva vestível que substitui o mouse convencional usando a orientação da mão e o toque dos dedos. Um **ESP32** lê os dados de movimento de um sensor **MPU6050** para mover o cursor com base na inclinação da mão (pitch/roll), enquanto os **pinos capacitivos nativos** do ESP32 detectam toques nos dedos para as ações de clique. O dispositivo se conecta a qualquer computador ou smartphone via **Bluetooth HID** — sem dongle USB, sem drivers.

Este projeto foi desenvolvido como Trabalho de Conclusão de Curso (TCC) na **Universidade de Vila Velha (UVV)**, Brasil.

---

## Motivação

O mouse tradicional exige uma superfície plana e movimentação lateral constante da mão, o que limita sua usabilidade em ambientes compactos, durante viagens ou em fluxos de trabalho que demandam alternância frequente entre teclado e mouse. O Glouse explora um modelo alternativo de interação: controlar o cursor pela inclinação natural da mão e pelo toque das pontas dos dedos, sem depender de nenhuma superfície.

---

## Índice

1. [Recursos](#recursos)
2. [Componentes](#componentes)
3. [Como Funciona](#como-funciona)
4. [Protótipos](#protótipos)
5. [Configuração de Pinos](#configuração-de-pinos)
6. [Primeiros Passos](#primeiros-passos)
7. [Status Atual](#status-atual)
8. [Licença](#licença)

---

## Recursos

- **Controle por Movimento** — O cursor se move com base no pitch e roll da mão via MPU6050
- **Botões Capacitivos** — Ações de clique pelos pinos capacitivos nativos do ESP32 (sem botões mecânicos)
- **Conectividade BLE HID** — Emparelha nativamente com computadores e smartphones como mouse Bluetooth padrão
- **Sem fio e Portátil** — Alimentado por bateria, sem cabos ou receptores USB
- **Multitarefa com FreeRTOS** — Leitura de sensores e transmissão BLE rodam como tarefas separadas para movimento suave do cursor
- **Sensibilidade Ajustável** — Sensibilidade do ponteiro e da rolagem configuráveis no código

---

## Componentes

### Hardware

| Componente | Função |
|---|---|
| ESP32 | Microcontrolador principal — processamento, BLE, toque capacitivo |
| MPU6050 | IMU de 6 eixos (acelerômetro + giroscópio) para orientação da mão |
| Bateria LiPo | Fonte de energia portátil |
| Módulo carregador + step-up | Gerenciamento de bateria (v2) |
| Display OLED | Exibição de status — bateria, modo (v2, em andamento) |

### Ambiente de Software

- PlatformIO no Visual Studio Code
- Framework Arduino para ESP32

### Bibliotecas

- [I2Cdev](https://github.com/jrowberg/i2cdevlib) — Comunicação I2C com o MPU6050
- [MPU6050_6Axis_MotionApps20](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) — Processamento de orientação via DMP
- [Wire](https://www.arduino.cc/en/Reference/Wire) — Biblioteca I2C para Arduino
- [BleMouse](https://github.com/T-vK/ESP32-BLE-Mouse) — Emulação de mouse BLE HID para ESP32

---

## Como Funciona

1. **Detecção de Movimento**
   - O MPU6050 lê continuamente dados do acelerômetro e do giroscópio
   - Os ângulos de pitch e roll são extraídos e mapeados para deslocamento X/Y do cursor
   - Fusão sensorial e filtragem reduzem ruído e evitam movimentos bruscos

2. **Controles por Toque**
   - Os pinos capacitivos do ESP32 funcionam como eletrodos em cada posição de dedo
   - O contato do dedo é detectado como variação de capacitância — sem botões físicos
   - Conectar o GND à pele do usuário estabiliza significativamente as leituras

3. **Transmissão BLE HID**
   - O ESP32 se apresenta ao sistema como um mouse Bluetooth HID padrão
   - Deltas de movimento e eventos de clique são enviados como relatórios HID com baixa latência
   - Compatível com Windows, macOS, Linux e Android nativamente

4. **Multitarefa com FreeRTOS**
   - A leitura dos sensores e a transmissão BLE rodam como tarefas separadas, garantindo movimento suave do cursor mesmo sob os atrasos de escalonamento do Bluetooth

---

## Protótipos

### Protótipo 1 — Prova de Conceito

A primeira versão validou todo o pipeline do sensor ao cursor em uma montagem com fios:
- ESP32 + MPU6050 + bateria 18650 + módulo de carga + conversor step-up
- Componentes fixados em uma luva comum com fita e cabos soltos
- **Resultado:** Movimento do cursor, detecção de cliques e emparelhamento BLE todos funcionais — porém pesado e com limitações ergonômicas pela bateria 18650

### Protótipo 2 — Versão Refinada

Construído sobre a funcionalidade confirmada do v1, com foco em ergonomia e portabilidade:
- Bateria LiPo menor e mais leve
- Carregador e step-up integrados em uma única placa compacta
- Layout dos componentes redistribuído na luva para maior conforto
- Redução de fios expostos
- Suporte para display OLED preparado para feedback de status (bateria, modo)

---

## Configuração de Pinos

| Pino ESP32 | Conexão | Componente | Função |
|---|---|---|---|
| 21 | SDA | MPU6050 | Dados I2C |
| 22 | SCL | MPU6050 | Clock I2C |
| 3.3V | VCC | MPU6050 | Alimentação |
| GND | GND | MPU6050 | Terra |
| GND | Contato com pele | Mão do usuário | Referência de terra para toque* |
| 15 | Dedo 1.1 | Luva | Clique Esquerdo |
| 13 | Dedo 1.2 | Luva | Voltar |
| 12 | Dedo 1.3 | Luva | Avançar |
| 14 | Dedo 2.1 | Luva | Clique Direito |
| 27 | Dedo 2.2 | Luva | Clique do Meio |
| 33 | Dedo 3.1 | Luva | Rolagem (especial) |
| 32 | Dedo 3.2 | Luva | Configuração (especial) |

**\*** Conectar o GND à pele do usuário fornece uma referência de capacitância estável, melhorando significativamente a precisão da detecção de toque.

---

## Primeiros Passos

### Pré-requisitos

- [PlatformIO](https://platformio.org/) instalado no Visual Studio Code
- Bibliotecas listadas acima (ou resolvidas automaticamente via `platformio.ini`)

### Configuração

1. Clone o repositório
2. Abra a pasta do projeto no VS Code com o PlatformIO
3. Conecte o ESP32 via USB
4. Compile e faça o upload pelo PlatformIO

### Uso

1. Ligue a luva
2. Emparelhe com seu computador ou smartphone via Bluetooth (nome do dispositivo: **"Glouse"**)
3. Use a inclinação da mão para mover o cursor e o toque dos dedos para clicar

---

## Status Atual

| Funcionalidade | Status |
|---|---|
| Conexão BLE HID | ✅ Funcionando |
| Movimento do cursor (pitch/roll) | ✅ Funcionando |
| Detecção de clique (capacitivo) | ✅ Funcionando |
| Gesto de rolagem | ✅ Funcionando |
| Display OLED de status | 🔄 Em andamento |
| Diagrama de fiação | 📋 Pendente |
| Interface de calibração de sensibilidade (no display) | 📋 Planejado |

> Este projeto está em desenvolvimento ativo. Algumas funcionalidades e detalhes de fiação podem mudar.

---

## Licença

Este projeto é open-source sob a [Licença MIT](LICENSE).

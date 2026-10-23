# DarkViz Robot — Pin Connections

## 1. Motor 1 + Encoder — Front

**Position:** Front  
**Motor Driver:** Motor Driver 2  
**Driver Channel:** A

### Motor 1 — Motor Connections

| Motor Connection | TB6612FNG Pin |
|---|---|
| Motor −ve | A02 |
| Motor +ve | A01 |

### Motor 1 — Hall Encoder Connections

| Encoder Connection | ESP32 |
|---|---|
| Hall Sensor VCC | 5V |
| Hall Sensor GND | GND |
| Hall Sensor A Vout | GPIO 36 |
| Hall Sensor B Vout | GPIO 39 |

---

## 2. Motor 2 + Encoder — Right

**Position:** Right / Back  
**Motor Driver:** Motor Driver 2  
**Driver Channel:** B

### Motor 2 — Motor Connections

| Motor Connection | TB6612FNG Pin |
|---|---|
| Motor −ve | B01 |
| Motor +ve | B02 |

### Motor 2 — Hall Encoder Connections

| Encoder Connection | ESP32 |
|---|---|
| Hall Sensor VCC | 5V |
| Hall Sensor GND | GND |
| Hall Sensor A Vout | GPIO 32 |
| Hall Sensor B Vout | GPIO 33 |

---

## 3. Motor 3 + Encoder — Left

**Position:** Left  
**Motor Driver:** Motor Driver 1  
**Driver Channel:** A

### Motor 3 — Motor Connections

| Motor Connection | TB6612FNG Pin |
|---|---|
| Motor −ve | A02 |
| Motor +ve | A01 |

### Motor 3 — Hall Encoder Connections

| Encoder Connection | ESP32 |
|---|---|
| Hall Sensor VCC | 5V |
| Hall Sensor GND | GND |
| Hall Sensor A Vout | GPIO 27 |
| Hall Sensor B Vout | GPIO 22 |

---

# 4. Motor Driver 1

**Physical position:** Right side

Motor Driver 1 controls **Motor 3** through Channel A.

### Power Connections

| TB6612FNG Pin | Connection |
|---|---|
| VM | Motor supply | 
| VCC | 5V |
| GND | ESP32 GND |
| STBY | 3.3V |

### Channel A — Motor 3

| TB6612FNG Pin | ESP32 / Motor |
|---|---|
| PWMA | GPIO 26 |
| AIN1 | GPIO 25 |
| AIN2 | GPIO 23 |
| A01 | Motor 3 +ve |
| A02 | Motor 3 −ve |

### Channel B

Channel B is **unused**.

| TB6612FNG Pin | Connection |
|---|---|
| BIN1 | GND |
| BIN2 | GND |
| PWMB | GND |
| B01 | Unused |
| B02 | Unused |

BIN1 and BIN2 are tied to GND. PWMB can be tied low to keep the unused channel disabled.

---

# 5. Motor Driver 2

**Physical position:** Left side

Motor Driver 2 controls:

- Motor 1 through Channel A
- Motor 2 through Channel B

### Power Connections

| TB6612FNG Pin | Connection |
|---|---|
| VM | Motor supply |
| VCC | 5V |
| GND | ESP32 GND |
| STBY | 3.3V |

---

## Channel A — Motor 1

| TB6612FNG Pin | ESP32 / Motor |
|---|---|
| PWMA | GPIO 16 |
| AIN1 | GPIO 17 |
| AIN2 | GPIO 04 |
| A01 | Motor 1 +ve |
| A02 | Motor 1 −ve |

---

## Channel B — Motor 2

| TB6612FNG Pin | ESP32 / Motor |
|---|---|
| PWMB | GPIO 13 |
| BIN1 | GPIO 19 |
| BIN2 | GPIO 21 |
| B01 | Motor 2 +ve |
| B02 | Motor 2 −ve |

---

# 6. ESP32 GPIO Summary

| GPIO | Function | Component |
|---:|---|---|
| GPIO 04 | AIN2 | Motor Driver 2 |
| GPIO 13 | PWMB | Motor Driver 2 |
| GPIO 16 | PWMA | Motor Driver 2 |
| GPIO 17 | AIN1 | Motor Driver 2 |
| GPIO 19 | BIN1 | Motor Driver 2 |
| GPIO 21 | BIN2 | Motor Driver 2 |
| GPIO 22 | Encoder B | Motor 3 |
| GPIO 23 | AIN2 | Motor Driver 1 |
| GPIO 25 | AIN1 | Motor Driver 1 |
| GPIO 26 | PWMA | Motor Driver 1 |
| GPIO 27 | Encoder A | Motor 3 |
| GPIO 32 | Encoder A | Motor 2 |
| GPIO 33 | Encoder B | Motor 2 |
| GPIO 36 | Encoder A | Motor 1 |
| GPIO 39 | Encoder B | Motor 1 |

---

# 7. Motor Control Summary

| Motor | Position | Driver | Channel | PWM | Direction 1 | Direction 2 |
|---|---|---|---|---|---|---|
| Motor 1 | Front | Driver 2 | A | GPIO 16 | GPIO 17 | GPIO 04 |
| Motor 2 | Right / Back | Driver 2 | B | GPIO 13 | GPIO 19 | GPIO 21 |
| Motor 3 | Left | Driver 1 | A | GPIO 26 | GPIO 25 | GPIO 23 |

---

# 8. Encoder Summary

| Motor | Encoder A | Encoder B | VCC | GND |
|---|---:|---:|---|---|
| Motor 1 | GPIO 36 | GPIO 39 | 3.3V | GND |
| Motor 2 | GPIO 32 | GPIO 33 | 3.3V | GND |
| Motor 3 | GPIO 27 | GPIO 22 | 3.3V | GND |

---


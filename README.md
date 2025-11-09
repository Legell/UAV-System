# UAV-System
Данный проект реализует **защищённую VPN-сеть** и систему передачи **MAVLink-телеметрии** от беспилотного летательного аппарата (БВС) к серверу управления.   Основная цель — надёжный обмен данными между **бортовым компьютером (Repka Pi/Nano Pi)** и **сервером Headscale**,   с дальнейшей обработкой и отображением данных через **веб-панель** или **QGroundControl**

## 🧱 Архитектура системы

```text
    ┌────────────────────────────┐
    │      Flight Controller     │
    │    (Pixhawk / ArduCopter)  │
    └────────────┬───────────────┘
                 │
             UART/USB
                 │
    ┌────────────────────────────┐
    │  Raspberry Pi (Repka-Pi)   │
    │  • MAVProxy в venv         │
    │  • Tailscale Client        │
    │  • GSM/LTE модем           │
    └────────────┬───────────────┘
                 │
         VPN-сеть Headscale
                 │
    ┌────────────────────────────┐
    │ Headscale Server (Linux)   │
    │  • Координатор VPN         │
    │  • Flask API Dashboard     │
    │  • Приём MAVLink-телеметрии│
    └────────────┬───────────────┘
                 │
           Ground Station
        (QGroundControl / Web UI)
```
---
## ⚙️ Основные компоненты

### 🔹 Бортовая часть (Repka-Pi)

| Компонент                                 | Назначение                        |
| ----------------------------------------- | --------------------------------- |
| 🧠 **Одноплатный компьютер Repka Pi (v3)** или **Nano Pi Neo**              | Центральный вычислительный узел   |
| 🕹 **Полетный контроллер Pixhawk или ArduCopter** | Генерирует MAVLink-телеметрию     |
| 🔌 **Интерфейс UART / USB**               | Передача данных    |
| 📡 **GSM модуль Sim7000E или LTE-модем OLAX F90**     | Доступ в сеть через сотовую связь |
| ⚡ **DC/DC 12→5 V**                        | Питание бортовых устройств        |
| 🧭 **GPS / ГЛОНАСС модуль**                      | Координаты и скорость БВС             |

<img width="650" height="700" alt="image_2025-11-10_01-20-02" src="https://github.com/user-attachments/assets/98af90a8-32fb-4efe-9999-531de204bbf9" /> <img width="350" height="400" alt="image_2025-11-10_01-19-45" src="https://github.com/user-attachments/assets/aa63f5b4-0f7d-4a99-8094-4a291a3b0294" />

---
### 🔹 Серверная часть

| Компонент                               | Назначение                       |
| --------------------------------------- | -------------------------------- |
| 💻 **Linux-сервер (Ubuntu/Debian)**     | Хостинг Headscale и Flask        |
| 🔐 **Headscale**                        | Self-hosted VPN для защищенного соединения БЛА и сервера|
| 🌐 **Flask Dashboard**                  | Веб-панель телеметрии            |
| 🔒 **SSL / OpenSSL**                    | HTTPS-шифрование                 |

---
## 🧰 Установка Headscale (сервер)

> 💡 Подходит для Ubuntu/Debian VPS/VDS с публичным IP (например, `109.123.165.213`)

### 1️⃣ Зависимости

```bash
sudo apt update && sudo apt install -y curl sqlite3 openssl
```

### 2️⃣ Установка Headscale

```bash
wget https://github.com/juanfont/headscale/releases/latest/download/headscale_amd64.deb
sudo dpkg -i headscale_amd64.deb
```
### 3️⃣ Конфигурация `/etc/headscale/config.yaml` для HeadScale
> 💡 В строке **server_url** значение **<HEADSCALE_SERVER_IP>** заменить на публичный IP вашей VPS/VDS (например, `109.123.165.213`)

```yaml
server_url: "https://<HEADSCALE_SERVER_IP>"
listen_addr: "0.0.0.0:443"

private_key_path: /var/lib/headscale/private.key
noise:
  private_key_path: /var/lib/headscale/noise_private.key

database:
  type: sqlite
  sqlite:
    path: /var/lib/headscale/db.sqlite

prefixes:
  v4: 100.64.0.0/10
  v6: fd7a:115c:a1e0::/48

derp:
  urls:
    - https://controlplane.tailscale.com/derpmap/default
  auto_update_enabled: true

dns:
  magic_dns: true
  base_domain: headscale.local
  nameservers:
    global:
      - 1.1.1.1
      - 8.8.8.8

tls_cert_path: /etc/ssl/headscale/server.crt
tls_key_path: /etc/ssl/headscale/server.key
```

### 4️⃣ Сертификаты ssl для подключения по https

```bash
sudo mkdir -p /etc/ssl/headscale
sudo openssl req -x509 -newkey rsa:4096 -sha256 -days 3650 -nodes \
  -keyout /etc/ssl/headscale/server.key \
  -out /etc/ssl/headscale/server.crt \
  -subj "/CN=<HEADSCALE_SERVER_IP>"
sudo chown headscale:headscale /etc/ssl/headscale/server.*
```



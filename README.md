
<!-- ===================================================================== -->

<!--                       README – can_listener_ros2                      -->

<!-- ===================================================================== -->

# Leitura de Barramento CAN com ValueCAN4-2 no ROS 2 Humble 

Projeto para realizar a **aquisição de dados da rede CAN** de veículos usando um **ValueCAN4-2** via **Intrepid SDK** e publicação em tópicos ROS 2.


<div align="center">

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://www.ros.org/)
[![Intrepid SDK](https://img.shields.io/badge/SDK-Intrepid%20Control%20Systems-green)](https://intrepidcs.com)

</div>

---

<div align="center">

• [Estrutura](#estrutura-do-repositório-📂)
• [Visão Geral](#visão-geral-🗺️)
• [O que os códigos fazem](#o-que-os-códigos-fazem-📝)
• [Resultados esperados](#resultados-esperados-✅)
• [Como interpretar os dados](#como-interpretar-os-dados-🔍)
• [Possíveis problemas](#possíveis-problemas-⚠️)
• [Como compilar e rodar](#como-compilar-e-rodar-🚀)
• [Licença](#licença-📄)

</div>

---

## Estrutura do repositório 📂

```bash
can_listener_ros2/
├── libs/
│   └── libicsneo/               # SDK Intrepid (ValueCAN4-2)
├── src/
│   ├── bag_to_file.cpp          # Script auxiliar: salva dados em arquivo
│   ├── can_listener.cpp         # Leitura CAN (polling)
│   └── can_listener_callback.cpp# Leitura CAN (callback)
├── CMakeLists.txt               # Configuração da build
├── package.xml                  # Metainformação do pacote
└── README.md                    # Este arquivo
```

---

## Visão geral 🗺️

Este pacote tem como objetivo **capturar mensagens da rede CAN** via ValueCAN4-2 e publicá-las em um tópico ROS 2.
A biblioteca **libicsneo** (SDK Intrepid) é usada para:

* Encontrar o dispositivo.
* Configurar baudrate e modo.
* Receber frames CAN.

O projeto oferece **duas formas de leitura**:

1. **Polling** (`can_listener.cpp`)
   Loop que periodicamente consulta se há novas mensagens.
2. **Callback** (`can_listener_callback.cpp`)
   Função chamada automaticamente a cada nova mensagem.

Ambos publicam dados brutos no tópico **`/can_data`** como `std_msgs/msg/UInt8MultiArray`.

---

## O que os códigos fazem 📝

### 🟢 1. `can_listener.cpp` (Leitura por Polling)

**Fluxo resumido:**

* Inicializa o ValueCAN4-2.
* Configura o canal HSCAN em baudrate 500 kbit/s e modo **listen-only**.
* Habilita coleta de mensagens.
* Entra em um loop de 10 Hz:

  * Verifica mensagens novas (`getMessages()`).
  * Publica o payload bruto da última mensagem lida no tópico `/can_data`.

**Características:**

* Latência baixa-média (depende da taxa de loop).
* Mais simples de entender.
* Útil para cenários onde se prefere processar lotes periodicamente.

---

### 🟢 2. `can_listener_callback.cpp` (Leitura via Callback)

**Fluxo resumido:**

* Inicializa o ValueCAN4-2.
* Configura baudrate e modo listen-only.
* Registra um **callback** com `addMessageCallback()`:

  * Sempre que chega uma nova mensagem, a função `processMessage()` é chamada.
  * Publica o payload imediatamente no tópico `/can_data`.

**Características:**

* Latência mínima (evento dispara instantaneamente).
* Mais eficiente em CPU.
* Ideal para sistemas reativos ou com grande volume de dados.

---

## Resultados esperados ✅

Independente do código escolhido, você terá:

* Tópico ROS 2 chamado `/can_data`.
* Mensagens do tipo `std_msgs/msg/UInt8MultiArray`.
* Cada mensagem contém **o vetor bruto de bytes** do frame CAN recebido.

Exemplo de saída (via `ros2 topic echo /can_data`):

```
data: [0, 255, 34, 12, 78, 90, 0, 1]
---
data: [255, 0, 1, 2]
---
```

---

## Como interpretar os dados 🔍

Cada `UInt8MultiArray` representa **o payload do frame CAN**, sem processamento adicional.

**Importante:**

* Não inclui ID da mensagem ou timestamp.
* Apenas o campo **data\[]** dos frames CAN.
* O tamanho pode variar conforme a mensagem original (DLC).

Se precisar do ID ou timestamp, posso alterar o pacote posteriormente.

---

## Possíveis problemas ⚠️

⚠️ Durante os testes, alguns efeitos colaterais foram observados ao conectar o ValueCAN4-2:

1. **Mídia do veículo piscando**

   * Pode ocorrer ao iniciar a leitura CAN.
   * Causa provável: variação de terminação da rede ou sondagem do barramento.

2. **Veículo desliga após a conexão**

   * Pode acontecer ao abrir o dispositivo ou configurar o baudrate.
   * Recomenda-se usar **modo listen-only** e sempre desabilitar terminação (`setTermination(false)`).

---

## Como compilar e rodar 🚀

Aqui vai o **passo a passo completo**, desde o clone até a execução:

### ✅ 1. Clonar o repositório

```bash
cd ~/ros2_ws/src
git clone https://github.com/Vinicius-GN/can_listener_ros2.git
```

---

### ✅ 2. Compilar o SDK Intrepid

Certifique-se de ter as dependências básicas:

```bash
sudo apt-get install build-essential cmake
```

Compile a biblioteca:

```bash
cd ~/ros2_ws/src/can_listener_ros2/libs/libicsneo
mkdir build
cd build
cmake ..
make
sudo make install
```

---

### ✅ 3. Instalar dependências ROS 2

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

---

### ✅ 4. Compilar o pacote ROS 2

```bash
colcon build
```

---

### ✅ 5. Preparar o ambiente

```bash
source install/setup.bash
```

---

### ✅ 6. Executar a leitura CAN

Escolha um dos nós:

▶️ **Leitura por Polling**

```bash
ros2 run can_listener_ros2 can_listener_node
```

▶️ **Leitura via Callback**

```bash
ros2 run can_listener_ros2 can_listener_callback_node
```

---

### ✅ 7. Verificar os dados capturados

Em outro terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /can_data
```

---

### ✅ 8. (Opcional) Gravar rosbag

Para gravar os dados:

```bash
ros2 bag record /can_data
```

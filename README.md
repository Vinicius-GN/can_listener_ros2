
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
## O que os códigos fazem:

### 🟢 1. `can_listener.cpp` (Leitura por Polling)

Este script implementa o fluxo de **leitura por polling**, de forma periódica.

### Resumo do funcionamento:

1. **Inicialização do ROS 2 e criação do nó**

   * Nó chamado `can_listener_node`.
   * Cria um publisher no tópico `/can_data` para enviar mensagens.

2. **Inicialização do dispositivo CAN**

   * Detecta se o ValueCAN4-2 está conectado.
   * Abre o dispositivo.
   * Configura:

     * Baudrate HSCAN: **500 kbit/s**.
     * Baudrate HSCAN2: **250 kbit/s**.
     * Modo listen-only: só escuta, não transmite.
     * Terminação: desativada.
   * Entra no modo online.
   * Habilita coleta de mensagens.

3. **Loop de aquisição**

   * Roda em frequência fixa de **10 Hz**.
   * Em cada iteração:

     * Chama `getMessages()` para ler todas as novas mensagens recebidas desde o último ciclo.
     * Verifica se são frames CAN.
     * Extrai o vetor de bytes da mensagem.
     * Publica no `/can_data`.

4. **Encerramento**

   * Ao finalizar (Ctrl+C), fecha o dispositivo e faz shutdown do ROS 2.

### 🧭 Quando usar:

Ideal para casos em que:

* Quer periodicidade fixa de leitura.
* Volume de mensagens é baixo a moderado.
* Precisa processar lotes de mensagens por ciclo.

---

### 🟢 2. `can_listener_callback.cpp` (Leitura via Callback)

Este script implementa o fluxo de **leitura assíncrona por callback**, reagindo imediatamente à chegada de mensagens.

### Resumo do funcionamento:

1. **Inicialização do ROS 2 e criação do nó**

   * A classe `CANReader` herda de `rclcpp::Node`.
   * Nó chamado `can_listener_callback_node`.
   * Cria publisher no tópico `/can_data`.

2. **Inicialização do dispositivo CAN**

   * Detecta e abre o ValueCAN4-2.
   * Configura:

     * Baudrate HSCAN: **500 kbit/s**.
     * Modo listen-only.
     * Terminação desativada.
   * Entra no modo online.
   * Habilita polling interno (buffer).
   * Registra um **callback**:

     * Toda vez que chega uma mensagem nova, o SDK chama automaticamente a função `processMessage()`.

3. **Callback `processMessage()`**

   * Verifica se a mensagem é frame CAN/SWCAN/LSFTCAN.
   * Converte o payload em vetor de bytes.
   * Publica no `/can_data` **imediatamente**.

4. **Loop de espera**

   * Ao invés de um loop manual, chama `rclcpp::spin()`:

     * Fica escutando eventos ROS 2 e callbacks do SDK.
     * Não há polling explícito.

5. **Encerramento**

   * Fecha o dispositivo ao sair.

### 🧭 Quando usar:

Ideal para casos em que:

* Precisa da menor latência possível.
* O volume de mensagens é alto.
* Quer reagir imediatamente ao tráfego CAN.

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

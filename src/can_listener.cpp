#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "icsneo/icsneocpp.h"
#include <vector>
#include <memory>

using namespace std;

// CLasse para gerenciar a comunicação com o ValueCAN4-2
class CANReader {
public:
  CANReader() : device(nullptr) {}

  ~CANReader() { // Destrutor por seguranca
    closeDevice();
  }

  bool initialize() {
    vector<shared_ptr<icsneo::Device>> devices = icsneo::FindAllDevices(); // Encontrar dispositivos
    if (devices.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("CANReader"), "Nenhum dispositivo CAN encontrado!");
      return false;
    }

    device = devices.front(); // Pega o primeiro elemento (dispositivo)

    // Tenta abrir o dispositivo
    if (!device->open()) { // Função de open retorna se o dispositivo foi aberto com sucesso
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CANReader"), "Falha ao abrir o dispositivo CAN: " << icsneo::GetLastError());
        return false;
    }
    
    //Setando as configurações do dispositivo para as configurações padrão
    // device->settings->refresh();
    // device->settings->apply();

    // Imprime informações do dispositivo
    RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Dispositivo conectado: %s", device->describe().c_str());
    RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Getting HSCAN e HSCAN2 Baudrate...");

    // auto* settings = device->settings.get();
    // RCLCPP_INFO(rclcpp::get_logger("CANReader"), settings);
    device->settings->setLINModeFor(icsneo::Network::NetID::LIN, SLEEP_MODE);
    device->settings->setCommanderResistorFor(icsneo::Network::NetID::LIN, false);

    int64_t baud = device->settings->getBaudrateFor(icsneo::Network::NetID::HSCAN);
    int64_t baud2 = device->settings->getBaudrateFor(icsneo::Network::NetID::HSCAN2);
    // int64_t baud3 = device->settings->getBaudrateFor(icsneo::Network::NetID::HSCAN3);
    // int64_t baud4 = device->settings->getBaudrateFor(icsneo::Network::NetID::HSCAN4);
		if(baud < 0)
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Falha ao pegar a baudRate");
		else
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Baudrate antes: %ld %ld", baud, baud2);

    // Definindo nova velocidade de leitura para ambos os canais
    baud = device->settings->setBaudrateFor(icsneo::Network::NetID::HSCAN, 500000);
    baud2 = device->settings->setBaudrateFor(icsneo::Network::NetID::HSCAN2, 250000);
    device->settings->apply();
		if(baud < 0)
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Falha ao setar a baudRate");
		else
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Baud Rate alterada com sucesso!");


    // Coloca o dispositivo online para começar a receber mensagens
    if (!device->goOnline()) {
      device->close();
      RCLCPP_ERROR(rclcpp::get_logger("CANReader"), "Falha ao colocar o dispositivo online!");
      return false;
    }

    // Habilita a coleta de mensagens no dispositivo
    device->enableMessagePolling();
    // device->setPollingMessageLimit(100000); // Caso queira limitar o número de mensagens recebidas

    return true;
  }

  vector<uint8_t> readMessages() {
    vector<uint8_t> dadosBinarios;
    vector<shared_ptr<icsneo::Message>> mensagens;
  
    if (device) {
      // Obtém apenas as mensagens novas; o buffer interno é removido (só publicar as novas mensagens)
      // -> Exige que o listener do node leia e salve em um buffer durante o processamento
      device->getMessages(mensagens);

      // Para cada mensagem recebida:
      for (const auto &msg : mensagens) {
        // Verifica se a mensagem é do tipo Frame
        if (msg->type == icsneo::Message::Type::Frame) {
          // Faz o cast direto para CANMessage
          auto canMsg = static_pointer_cast<icsneo::CANMessage>(msg);
          // canmsg->arbid is valid here
          // canmsg->data is an std::vector<uint8_t>, you can check .size() for the DLC of the message
          // canmsg->timestamp is the time recorded by the hardware in nanoseconds since (1/1/2007 12:00:00 GMT)
          
          // Acrescenta os bytes da mensagem à string para publicar a mensagem sem processamento no ROS2
          dadosBinarios= canMsg->data;
        }
      }
    }
    return dadosBinarios;
  }
  
  void closeDevice() {
    if (device) {
      device->close();
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Dispositivo fechado com sucesso.");
    }
  }

private:
  shared_ptr<icsneo::Device> device;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Cria um nó ROS2 com nome "can_listener_node"
  auto node = rclcpp::Node::make_shared("can_listener_node");

  // Cria o publicador no tópico "can_data"
  auto can_pub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("can_data", 10);

  CANReader canReader;
  if (!canReader.initialize()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Rate loop_rate(10); // Frequência de 10 Hz
  
  while (rclcpp::ok()) { // Loop de leitura e publicação das mensagens
    vector<uint8_t> messages = canReader.readMessages();
    std_msgs::msg::UInt8MultiArray ros_msg;
    ros_msg.data = messages;
    can_pub->publish(ros_msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  canReader.closeDevice();
  rclcpp::shutdown();
  return 0;
}

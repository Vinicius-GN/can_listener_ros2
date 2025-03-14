#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
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

    device = devices.front(); // Permitir a interação com o dispositivo

    // Tenta abrir o dispositivo
    if (!device->open()) { // Função de open retorna se o dispositivo foi aberto com sucesso
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CANReader"), "Falha ao abrir o dispositivo CAN: " << icsneo::GetLastError());
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Dispositivo conectado: %s", device->getProductName().c_str());

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

  string readMessages() {
    string dadosBinarios;
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
          dadosBinarios.append(reinterpret_cast<const char*>(canMsg->data.data()),
                               canMsg->data.size());
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
  auto can_pub = node->create_publisher<std_msgs::msg::String>("can_data", 10);

  CANReader canReader;
  if (!canReader.initialize()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Rate loop_rate(10); // Frequência de 10 Hz
  while (rclcpp::ok()) { // Loop de leitura e publicação das mensagens
    string messages = canReader.readMessages();
    std_msgs::msg::String ros_msg;
    ros_msg.data = messages;
    can_pub->publish(ros_msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  canReader.closeDevice();
  rclcpp::shutdown();
  return 0;
}

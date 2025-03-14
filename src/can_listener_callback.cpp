#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "icsneo/icsneocpp.h"
#include <vector>
#include <memory>

using namespace std;

// Classe para gerenciar a comunicação com o ValueCAN4-2
class CANReader : public rclcpp::Node {
public:
  // Construtor: define o nome do nó como "can_listener_callback_node"
  CANReader() : Node("can_listener_callback_node"), device(nullptr) {
    // Configura o publicador no ROS2
    can_pub = this->create_publisher<std_msgs::msg::String>("can_data", 10);
  }

  ~CANReader() { // Destrutor da classe, por segurança
    closeDevice();
  }

  bool initialize() {
    // Encontra dispositivos
    vector<shared_ptr<icsneo::Device>> devices = icsneo::FindAllDevices();
    if (devices.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Nenhum dispositivo CAN encontrado!");
      return false;
    }
    device = devices.front(); // Permite a interação com um dispositivo
    if (!device->open()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CANReader"), "Falha ao abrir o dispositivo CAN: " << icsneo::GetLastError());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Dispositivo conectado: %s", device->getProductName().c_str());
    if (!device->goOnline()) { // Coloca o dispositivo online para começar a receber mensagens
      device->close();
      RCLCPP_ERROR(this->get_logger(), "Falha ao colocar o dispositivo online");
      return false;
    }

    device->enableMessagePolling(); // Habilita a coleta de mensagens no dispositivo
    device->addMessageCallback(std::make_shared<icsneo::MessageCallback>(
      [this](std::shared_ptr<icsneo::Message> message) {
        this->processMessage(message);
      }
    ));

    return true;
  }

private:
  shared_ptr<icsneo::Device> device;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr can_pub;

  void processMessage(std::shared_ptr<icsneo::Message> msg) {
    // Verifica se a mensagem é do tipo Frame
    if (msg->type == icsneo::Message::Type::Frame) {

        auto frame = std::static_pointer_cast<icsneo::Frame>(msg);
      //Verifica o tipo de rede usando frame->network.getType()
      
      if (frame->network.getType() == icsneo::Network::Type::CAN ||
          frame->network.getType() == icsneo::Network::Type::SWCAN ||
          frame->network.getType() == icsneo::Network::Type::LSFTCAN) {
        
        // Como é uma mensagem CAN, faz o cast para CANMessage
        auto canmsg = std::static_pointer_cast<icsneo::CANMessage>(msg);
        // canmsg->arbid is valid here
        // canmsg->data is an std::vector<uint8_t>, you can check .size() for the DLC of the message
        // canmsg->timestamp is the time recorded by the hardware in nanoseconds since (1/1/2007 12:00:00 GMT)
      
        // Converte os dados binários diretamente para uma std::string
        std::string binary_data(reinterpret_cast<const char*>(canmsg->data.data()), canmsg->data.size());
        std_msgs::msg::String ros_msg;
        ros_msg.data = binary_data;
        can_pub->publish(ros_msg);
      }
    }
  }
  

  void closeDevice() {
    if (device) {
      device->close();
      RCLCPP_INFO(this->get_logger(), "Dispositivo fechado com sucesso.");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CANReader>();
  if (!node->initialize()) {
    rclcpp::shutdown();
    return 1;
  }

  // Mantém o nó ROS2 ativo
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
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
    can_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("can_data", 10);
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

     RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Getting HSCAN Baudrate...");

    int64_t baud = device->settings->getBaudrateFor(icsneo::Network::NetID::HSCAN);
		if(baud < 0)
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Failha ao pegat a baudRate");
		else
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Baudrate antes: %ld", baud);

    baud = device->settings->setBaudrateFor(icsneo::Network::NetID::HSCAN, 500000);
    device->settings->apply();

		if(baud < 0)
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Falha ao setar a baudRate");
		else
      RCLCPP_INFO(rclcpp::get_logger("CANReader"), "Baudrate depois: %ld", baud);

    RCLCPP_INFO(this->get_logger(), "Dispositivo conectado: %s", device->describe().c_str());
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
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr can_pub;

  void processMessage(std::shared_ptr<icsneo::Message> msg) {
    // Verifica se a mensagem é do tipo Frame
    if (msg->type == icsneo::Message::Type::Frame) {

        auto frame = std::static_pointer_cast<icsneo::Frame>(msg);

      // Verifica o tipo de rede usando frame->network.getType()
      if (frame->network.getType() == icsneo::Network::Type::CAN ||
          frame->network.getType() == icsneo::Network::Type::SWCAN ||
          frame->network.getType() == icsneo::Network::Type::LSFTCAN) {

        // Como é uma mensagem CAN, faz o cast para CANMessage
        auto canmsg = std::static_pointer_cast<icsneo::CANMessage>(msg);
        // canmsg->arbid is valid here
        // canmsg->data is an std::vector<uint8_t>, you can check .size() for the DLC of the message
        // canmsg->timestamp is the time recorded by the hardware in nanoseconds since (1/1/2007 12:00:00 GMT)
      
        // Converte os dados binários diretamente para uma std::vector<uint8_t>
        vector<uint8_t> bytes_data = canmsg->data;
        std_msgs::msg::UInt8MultiArray ros_msg;
        ros_msg.data = bytes_data;
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

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

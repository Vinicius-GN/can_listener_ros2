#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <fstream>
#include <sstream>
#include <memory>

class CanDataLogger : public rclcpp::Node
{
public:
  CanDataLogger()
  : Node("can_data_logger")
  {
    // Cria a inscrição para o tópico "can_data" com fila de 10 mensagens
    subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "can_data", 10,
      std::bind(&CanDataLogger::topic_callback, this, std::placeholders::_1));

    // Abre o arquivo para escrita; se já existir, sobrescreve
    output_file_.open("output.txt");
    if (!output_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Falha ao abrir output.txt para escrita");
    } else {
      RCLCPP_INFO(this->get_logger(), "Arquivo output.txt aberto para escrita");
    }
  }

  ~CanDataLogger()
  {
    // Fecha o arquivo ao destruir o nó
    if (output_file_.is_open()) {
      output_file_.close();
      RCLCPP_INFO(this->get_logger(), "Arquivo output.txt salvo e fechado");
    }
  }

private:
  void topic_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    // Converte os dados (vetor de uint8) em uma string com os valores separados por espaço
    std::stringstream ss;
    for (const auto &val : msg->data) {
      ss << static_cast<int>(val) << " ";
    }
    std::string line = ss.str();

    // Escreve a linha no arquivo com quebra de linha
    if (output_file_.is_open()) {
      output_file_ << line << "\n";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Arquivo de saída não está aberto!");
    }
  }

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  std::ofstream output_file_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanDataLogger>();

  // O nó permanecerá ativo até ser interrompido com Ctrl+C pelo usuário
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "ros/ros.h"
#include "TangentBug.h"

int main(int argc, char **argv)
{
    // 1. Inicializa o ROS
    // O primeiro argumento é o número de argumentos de linha de comando.
    // O segundo argumento é o vetor de argumentos de linha de comando.
    // O terceiro argumento é o nome do nó ROS. Este nome deve ser único no seu grafo ROS.
    ros::init(argc, argv, "tangent_bug_node");

    // 2. Cria um NodeHandle
    // O NodeHandle é a interface principal para a comunicação com o resto do sistema ROS.
    // Ele é necessário para criar publishers, subscribers, services, etc.
    ros::NodeHandle nh;

    // 3. Instancia a classe TangentBug
    // Passa o NodeHandle para a construtora da sua classe.
    // A construtora de TangentBug irá configurar os subscribers, publishers e o timer de controle.
    TangentBug bug(nh);

    // 4. Inicia o módulo de navegação (opcional, pode ser apenas uma chamada de inicialização)
    // Se a sua função `Maps()` não tiver mais um `while(ros::ok())` interno,
    // ela pode ser chamada aqui para configurar qualquer estado inicial ou log.
    bug.navigate();

    // 5. Entra no loop principal do ROS
    // `ros::spin()` entra em um loop e processa todos os callbacks pendentes
    // (como os callbacks de subscribers e o callback do timer de `computeControl`).
    // Ele só retorna quando `ros::ok()` se torna falso (por exemplo, Ctrl+C no terminal).
    ros::spin();

    // 6. Retorna 0 ao sair do loop (indica sucesso)
    return 0;
}

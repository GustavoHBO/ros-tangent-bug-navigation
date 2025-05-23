#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h> // Pode não ser mais necessário, dependendo de outras partes de TangentBug
#include <limits>
#include "../src/TangentBug.h" // Verifique se o caminho está correto

class FindClosestEndpointTest : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    TangentBug bug; // A classe TangentBug agora deve ter a função findClosestEndpointToPoint com a nova assinatura

    FindClosestEndpointTest() : nh_("~"), bug(nh_) {}

    geometry_msgs::Point makePoint(double x, double y) {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;
        return pt;
    }
};

// Teste para um segmento simples onde o último ponto é o mais próximo
TEST_F(FindClosestEndpointTest, ReturnsClosestFromSingleSegment)
{
    // Agora passamos apenas um segmento
    std::vector<geometry_msgs::Point> segment = { makePoint(1, 1), makePoint(2, 2), makePoint(3, 3) };
    geometry_msgs::Point target = makePoint(3.2, 3.1);

    // Chamada da função com a nova assinatura
    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 3.0, 1e-3);
    EXPECT_NEAR(closest.y, 3.0, 1e-3);
}

// Teste para um segmento simples onde o primeiro ponto é o mais próximo
TEST_F(FindClosestEndpointTest, SelectsFirstWhenCloserThanLast)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(0, 0), makePoint(5, 5) };
    geometry_msgs::Point target = makePoint(0.1, 0.1);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}

// Este teste agora é inválido, pois a função não trabalha mais com múltiplos segmentos.
// Você precisará decidir se quer testar a lógica que combina vários segmentos em outro lugar,
// ou se esta funcionalidade não é mais necessária para TangentBug.
/*
TEST_F(FindClosestEndpointTest, WorksWithMultipleSegments)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(10, 10), makePoint(11, 11) },
        { makePoint(0, 0), makePoint(1, 1) },
        { makePoint(5, 5), makePoint(6, 6) }
    };

    geometry_msgs::Point target = makePoint(0.5, 0.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}
*/

// Teste para lidar com segmento vazio
TEST_F(FindClosestEndpointTest, HandlesEmptySegmentGracefully)
{
    std::vector<geometry_msgs::Point> segment = {}; // Segmento vazio
    geometry_msgs::Point target = makePoint(2.1, 2.0);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    // Esperamos que retorne um ponto padrão (0,0,0) para um segmento vazio
    EXPECT_EQ(closest.x, 0.0);
    EXPECT_EQ(closest.y, 0.0);
}

// Teste para caso de equidistância (deve escolher o primeiro ponto por <=)
TEST_F(FindClosestEndpointTest, ReturnsFirstWhenEquidistant)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(1, 1), makePoint(4, 4) };
    geometry_msgs::Point target = makePoint(1, 1);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 1.0, 1e-3);
    EXPECT_NEAR(closest.y, 1.0, 1e-3);
}

// Este teste também é inválido, pois a função agora recebe apenas um segmento e lida
// com o caso de ser vazio retornando um ponto (0,0,0).
/*
TEST_F(FindClosestEndpointTest, ReturnsZeroForAllEmpty)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        {}, {}, {}
    };

    geometry_msgs::Point target = makePoint(1, 1);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_EQ(closest.x, 0.0);
    EXPECT_EQ(closest.y, 0.0);
}
*/

// Teste para coordenadas negativas
TEST_F(FindClosestEndpointTest, HandlesNegativeCoordinates)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(-1, -1), makePoint(-2, -2) };
    geometry_msgs::Point target = makePoint(-1.5, -1.5);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, -1.0, 1e-3);
    EXPECT_NEAR(closest.y, -1.0, 1e-3);
}

// Teste para coordenadas grandes
TEST_F(FindClosestEndpointTest, HandlesLargeCoordinates)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(10000, 10000), makePoint(20000, 20000) };
    geometry_msgs::Point target = makePoint(15000, 15000);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 10000.0, 1e-3);
    EXPECT_NEAR(closest.y, 10000.0, 1e-3);
}

// Teste para segmento com apenas um ponto (primeiro e último são o mesmo)
TEST_F(FindClosestEndpointTest, HandlesSinglePointSegment)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(1, 1) };
    geometry_msgs::Point target = makePoint(1.1, 1.1);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 1.0, 1e-3);
    EXPECT_NEAR(closest.y, 1.0, 1e-3);
}

// Este teste é inválido. A função não compara entre *múltiplos* segmentos.
/*
TEST_F(FindClosestEndpointTest, HandlesMultipleSegmentsWithSameDistance)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(2, 2) },
        { makePoint(3, 3), makePoint(4, 4) }
    };

    geometry_msgs::Point target = makePoint(2.5, 2.5);
    auto closest = bug.findClosestEndpointToPoint(segments, target);

    EXPECT_NEAR(closest.x, 2.0, 1e-3);
    EXPECT_NEAR(closest.y, 2.0, 1e-3);
    // Check if the first segment is chosen
    EXPECT_EQ(closest.x, segments[0][1].x);
    EXPECT_EQ(closest.y, segments[0][1].y);
}
*/

// Teste para segmento longo com alvo próximo de um dos extremos
TEST_F(FindClosestEndpointTest, HandlesLongSegmentWithCloseTarget)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(0, 0), makePoint(100, 100) };
    geometry_msgs::Point target = makePoint(50, 50); // Mais próximo de (0,0) do que (100,100)

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}

// Teste para segmento com coordenadas negativas
TEST_F(FindClosestEndpointTest, HandlesSegmentWithNegativeCoordinates)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(-10, -10), makePoint(-20, -20) };
    geometry_msgs::Point target = makePoint(-15, -15); // Mais próximo de (-10,-10)

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, -10.0, 1e-3);
    EXPECT_NEAR(closest.y, -10.0, 1e-3);
}

// Teste para segmento com coordenadas mistas (positivas e negativas)
TEST_F(FindClosestEndpointTest, HandlesSegmentWithMixedCoordinates)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(-10, 10), makePoint(20, -20) };
    geometry_msgs::Point target = makePoint(5, 5); // Mais próximo de (-10, 10)

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, -10.0, 1e-3);
    EXPECT_NEAR(closest.y, 10.0, 1e-3);
}

// Teste para segmento com comprimento zero (primeiro e último ponto são iguais)
TEST_F(FindClosestEndpointTest, HandlesSegmentWithZeroLength)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(1, 1), makePoint(1, 1) };
    geometry_msgs::Point target = makePoint(1.1, 1.1);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 1.0, 1e-3);
    EXPECT_NEAR(closest.y, 1.0, 1e-3);
}

// Os testes abaixo, que usavam "multiple close points" e "multiple segments",
// precisam ser reavaliados. A função agora ignora pontos intermediários e
// só considera os extremos do *único* segmento.
// Se a intenção é ter um segmento com muitos pontos e garantir que apenas os extremos
// são considerados, os testes podem ser adaptados.
// Se a intenção era testar a lógica de busca em *todos os pontos* de múltiplos segmentos,
// essa funcionalidade não está mais na função que você editou.

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultiplePoints_TargetCloserToLast)
{
    // A função agora só olha para makePoint(1,1) e makePoint(3,3).
    // O ponto intermediário (2,2) é ignorado para o cálculo do mais próximo.
    std::vector<geometry_msgs::Point> segment = { makePoint(1, 1), makePoint(2, 2), makePoint(3, 3) };
    geometry_msgs::Point target = makePoint(2.5, 2.5); // Target é (2.5, 2.5)
                                                      // Distância de (1,1) a (2.5,2.5) é ~2.12
                                                      // Distância de (3,3) a (2.5,2.5) é ~0.707
    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 3.0, 1e-3);
    EXPECT_NEAR(closest.y, 3.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultiplePointsAndNegativeCoordinates_TargetCloserToLast)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(-1, -1), makePoint(-2, -2), makePoint(-3, -3) };
    geometry_msgs::Point target = makePoint(-2.5, -2.5);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, -3.0, 1e-3);
    EXPECT_NEAR(closest.y, -3.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultiplePointsAndLargeCoordinates_TargetCloserToFirst)
{
    std::vector<geometry_msgs::Point> segment = { makePoint(1000, 1000), makePoint(2000, 2000), makePoint(3000, 3000) };
    geometry_msgs::Point target = makePoint(1500, 1500);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 1000.0, 1e-3);
    EXPECT_NEAR(closest.y, 1000.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultiplePointsAndNegativeLargeCoordinates_TargetCloserToFirst)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(-1000, -1000), makePoint(-2000, -2000), makePoint(-3000, -3000)
    };
    geometry_msgs::Point target = makePoint(-1500, -1500);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, -1000.0, 1e-3);
    EXPECT_NEAR(closest.y, -1000.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultiplePointsAndMixedCoordinates_TargetCloserToLast)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(-1000, 1000), makePoint(2000, -2000), makePoint(3000, 3000)
    };
    geometry_msgs::Point target = makePoint(1500, 1500); // Target é (1500, 1500)
                                                      // Distância de (-1000,1000) a (1500,1500) é sqrt(2500^2 + 500^2) = ~2549
                                                      // Distância de (3000,3000) a (1500,1500) é sqrt(1500^2 + 1500^2) = ~2121
    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 3000.0, 1e-3);
    EXPECT_NEAR(closest.y, 3000.0, 1e-3);
}

TEST_F(FindClosestEndpointTest, HandlesSegmentWithMultiplePointsAndZeroCoordinates_TargetCloserToFirst)
{
    std::vector<geometry_msgs::Point> segment = {
        makePoint(0, 0), makePoint(1, 1), makePoint(2, 2)
    };
    geometry_msgs::Point target = makePoint(0.5, 0.5);

    auto closest = bug.findClosestEndpointToPoint(segment, target);

    EXPECT_NEAR(closest.x, 0.0, 1e-3);
    EXPECT_NEAR(closest.y, 0.0, 1e-3);
}

// Este teste, que antes verificava múltiplos segmentos, precisa ser reescrito
// ou removido, pois a função agora só aceita um segmento.
// Se você ainda precisa de uma função que encontre o ponto mais próximo entre *todos* os
// endpoints de *múltiplos* segmentos, essa lógica deve estar em uma nova função
// (ou a original `findClosestEndpointToPoint` precisa ser restaurada com um nome diferente).
/*
TEST_F(FindClosestEndpointTest, HandlesSegmenstWithMultiplePoints)
{
    std::vector<std::vector<geometry_msgs::Point>> segments = {
        { makePoint(1, 1), makePoint(2, 2), makePoint(3, 3), makePoint(4, 4) , makePoint(5, 5), makePoint(6, 6) },
        { makePoint(7, 7), makePoint(8, 8), makePoint(9, 9), makePoint(10, 10) },
        { makePoint(11, 11), makePoint(12, 12), makePoint(13, 13), makePoint(14, 14) },
        { makePoint(15, 15), makePoint(16, 16), makePoint(17, 17), makePoint(18, 18) }
    };
    geometry_msgs::Point target = makePoint(10, 10);
    auto closest = bug.findClosestEndpointToPoint(segments, target);
    EXPECT_NEAR(closest.x, 10.0, 1e-3);
    EXPECT_NEAR(closest.y, 10.0, 1e-3);
    // Check if the first segment is chosen
    EXPECT_EQ(closest.x, segments[1][3].x);
    EXPECT_EQ(closest.y, segments[1][3].y);

    target = makePoint(5, 5);
    closest = bug.findClosestEndpointToPoint(segments, target);
    EXPECT_NEAR(closest.x, 6.0, 1e-3);
    EXPECT_NEAR(closest.y, 6.0, 1e-3);
    // Check if the first segment is chosen
    EXPECT_EQ(closest.x, segments[0][5].x);
    EXPECT_EQ(closest.y, segments[0][5].y);
}
*/

// === ENTRY POINT ===

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_closest_endpoint_to_point"); // Nome do nó do ROS mais descritivo para o teste
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
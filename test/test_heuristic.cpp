#include <gtest/gtest.h>

#include "../include/graph.hpp"
/*----------------------------------------------------------*/

TEST(EdgePositionTest, EdgePositionTest) {
  EdgePosition p1 = EdgePosition(0, 10);
  EdgePosition p2 = EdgePosition(1, -5);
  EdgePosition p3 = EdgePosition(0, -5);

  ASSERT_TRUE(p1.is_left_right());
  ASSERT_FALSE(p1.is_right_left());
  ASSERT_TRUE(p2.is_right_left());
  ASSERT_FALSE(p2.is_left_right());

  ASSERT_EQ(p1.direction(), 1);
  ASSERT_EQ(p2.direction(), -1);

  ASSERT_FALSE(p1 == p2);
  ASSERT_FALSE(p1 == p3);
  ASSERT_FALSE(p2 == p3);
  ASSERT_TRUE(p1 == p1);
}
/*----------------------------------------------------------*/

TEST(ChainTest, ChainTest) {
  Chain c1 = Chain({0, 0}, {0, 10});
  Chain c2 = Chain({0, 0}, {1, 5});

  ASSERT_TRUE(c1.is_single_edge());
  ASSERT_FALSE(c2.is_single_edge());

  ASSERT_FALSE(c1 == c2);
}
/*----------------------------------------------------------*/

class GraphTest : public ::testing::Test {

public:
  Graph straight;
  Graph reverse_station;

protected:
  virtual void SetUp() override {
    straight = Graph(6);
    straight.add_edge(0, 1, 100, 10); // edge 0
    straight.add_edge(1, 2, 100, 10); // edge 1
    straight.add_edge(2, 3, 100, 10); // edge 2
    straight.add_edge(3, 4, 100, 10); // edge 3
    straight.add_edge(4, 5, 100, 10); // edge 4

    reverse_station = Graph(8);
    reverse_station.add_edge(0, 1, 100, 10); // edge 0
    reverse_station.add_edge(1, 2, 100, 10); // edge 1
    reverse_station.add_edge(2, 3, 100, 10); // edge 2
    reverse_station.add_edge(3, 4, 100, 10); // edge 3
    reverse_station.add_edge(4, 5, 100, 10); // edge 4
    reverse_station.add_edge(4, 6, 100, 10); // edge 5
    reverse_station.add_edge(6, 7, 100, 10); // edge 6
    reverse_station.add_edge(7, 2, 100, 10); // edge 7
    reverse_station.invalidate_turn(2, 7);
    reverse_station.invalidate_turn(3, 5);
  }
};

TEST_F(GraphTest, ConnectivityTest) {
  ASSERT_TRUE(straight.connects_right(0, 1));
  ASSERT_FALSE(straight.connects_left(0, 1));
  ASSERT_FALSE(straight.connects_left(0, 2));
  ASSERT_TRUE(reverse_station.connects_left(4, 5));

  ASSERT_EQ(straight.connectivity(0, 1), 1);
  ASSERT_EQ(straight.connectivity(1, 0), 1);
  ASSERT_EQ(straight.connectivity(0, 0), -1);
  ASSERT_EQ(straight.connectivity(0, 3), 0);
  ASSERT_EQ(reverse_station.connectivity(1, 7), -1);
  ASSERT_EQ(reverse_station.connectivity(7, 1), -1);
  ASSERT_EQ(reverse_station.connectivity(4, 5), -1);

  ASSERT_EQ(straight.left_adjacent(0), std::vector<Edge>{});
  ASSERT_EQ(straight.left_adjacent(1), std::vector<Edge>{0});
  ASSERT_EQ(straight.right_adjacent(0), std::vector<Edge>{1});
  ASSERT_EQ(reverse_station.adjacent_edges(7), std::vector<Edge>({6, 1}));
  ASSERT_EQ(reverse_station.left_adjacent(7), std::vector<Edge>{6});
  ASSERT_EQ(reverse_station.right_adjacent(7), std::vector<Edge>{1});
}

TEST_F(GraphTest, AccessTest) { ASSERT_EQ(straight.num_edges(), 5); }

TEST_F(GraphTest, SubGraphTest) {}

TEST_F(GraphTest, CollisionTest) {
  Chain c1{{2, 0}, {3, 50}}, c2{{2, 50}, {3, 80}},
      c3{{1, 60}, {4, 100}, std::vector<Edge>({2, 3})}, c4{{2, -10}, {2, -30}},
      c5{{3, -70}, {2, -20}}, c6{{3, -10}, {3, -30}}, c7{{2, 0}, {2, 50}}, c8 {
    {0, 0},{0, -100}}, c9{{0, -99}};
    
  ASSERT_TRUE(straight.collision(c1, c2));
  ASSERT_TRUE(straight.collision(c2, c1));
  ASSERT_TRUE(straight.collision(c1, c1));
  ASSERT_TRUE(straight.collision(c1, c3));
  ASSERT_TRUE(straight.collision(c2, c3));
  ASSERT_TRUE(straight.collision(c3, c2));
  ASSERT_TRUE(straight.collision(c3, c1));
  ASSERT_TRUE(straight.collision(c2, c2));
  ASSERT_TRUE(straight.collision(c3, c3));

  ASSERT_TRUE(straight.collision(c1, c4));
  ASSERT_TRUE(straight.collision(c1, c5));
  ASSERT_TRUE(straight.collision(c4, c5));
  ASSERT_FALSE(straight.collision(c1, c6));
  ASSERT_TRUE(straight.collision(c1, c1));
  ASSERT_FALSE(straight.collision(c2, c7));

  Graph test_graph(8);
  test_graph.add_edge(0, 1, 100, 100);
  test_graph.add_edge(2, 1, 100, 100);
  test_graph.add_edge(3, 1, 100, 100);
  test_graph.add_edge(1, 4, 1000, 5000);
  test_graph.add_edge(4, 5, 100, 100);
  test_graph.add_edge(4, 6, 100, 100);
  test_graph.add_edge(4, 7, 100, 100);
  Chain test_c1({1, 100}, {3,1000});
  Chain test_c2({3,1000}, {4,100});
  Chain test_c3({2, 100}, {3,0});
  ASSERT_FALSE(test_graph.collision(test_c3, test_c2));
  ASSERT_FALSE(test_graph.collision(test_c1, test_c2));
  
  ASSERT_TRUE(test_graph.collision(c8, c9));
  // TODO: probably needs some more testing to be completely robust
}

/*----------------------------------------------------------*/

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

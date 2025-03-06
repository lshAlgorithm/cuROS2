#include "rclcpp/rclcpp.hpp"
#include <vector>

extern void launchVectorAdd(const float *A, const float *B, float *C, int N);

class VectorAddNode : public rclcpp::Node {
public:
    VectorAddNode() : Node("vectoradd_node") {
        int N = 10;
        std::vector<float> A(N, 1.0f), B(N, 2.0f), C(N);

        launchVectorAdd(A.data(), B.data(), C.data(), N);

        RCLCPP_INFO(this->get_logger(), "Vector Addition Result:");
        for (int i = 0; i < N; ++i) {
            RCLCPP_INFO(this->get_logger(), "C[%d] = %f", i, C[i]);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorAddNode>());
    rclcpp::shutdown();
    return 0;
}
